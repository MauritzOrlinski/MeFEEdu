mod fem;
mod solver;
mod vector2;

use crate::fem::BeamSimulation;
use clap::Parser;
use raylib::prelude::Camera2D;
use raylib::prelude::RaylibDraw;
use raylib::prelude::RaylibMode2DExt;
use raylib::prelude::Vector2 as RaylibVector2;
use raylib::{prelude::Color, prelude::RaylibDrawHandle};
use std::fs;
use vector2::Vector2;

#[derive(Parser, Debug)]
#[command(version, about, long_about = "FEEEEEEED ME a file")]
struct Args {
    #[arg(short, long)]
    file_path: String,
    #[arg(short, long, default_value_t = 1000)]
    maximum_iterations: usize,
    #[arg(short, long, default_value_t = 1E-15)]
    error_constraint: f64,
}

const H: i32 = 1000;
const W: i32 = 800;

const RADIUS: f32 = 5.0;
const BACKGROUND: Color = Color::BLACK;
const NODE_COLOR: Color = Color::BLUE;
const EDGE_COLOR: Color = Color::WHITE;
const STABLE_NODE_COLOR: Color = Color::RED;
#[allow(dead_code)]
fn draw_polygon_filled(d: &mut RaylibDrawHandle, points: &[RaylibVector2], color: Color) {
    if points.len() < 3 {
        return;
    }

    let center = points
        .iter()
        .fold(RaylibVector2 { x: 0.0, y: 0.0 }, |acc, p| acc + *p)
        / (points.len() as f32);

    for i in 0..points.len() {
        let p1 = points[i];
        let p2 = points[(i + 1) % points.len()];
        d.draw_triangle(center, p1, p2, color);
    }
}

fn to_display(v: &vector2::Vector2) -> raylib::ffi::Vector2 {
    raylib::ffi::Vector2 {
        x: v.x as f32,
        y: -v.y as f32,
    }
}

fn get_camera<'a, I>(mut points: I) -> Camera2D
where
    I: Iterator<Item = &'a Vector2>,
{
    let first = points
        .next()
        .expect("Point list must at least contain one point");

    let (x_axis, y_axis) = points.fold(
        ((first.x, first.x), (first.y, first.y)),
        |((x_min, x_max), (y_min, y_max)), point| {
            (
                (x_min.min(point.x), x_max.max(point.x)),
                (y_min.min(point.y), y_max.max(point.y)),
            )
        },
    );

    let (x_min, x_max) = x_axis;
    let x_extent = x_max - x_min;

    let (y_min, y_max) = y_axis;
    let y_extent = y_max - y_min;
    let max_x_zoom = (W as f64) / x_extent;
    let max_y_zoom = (H as f64) / y_extent;

    let zoom = max_y_zoom.min(max_x_zoom) as f32;
    let position = RaylibVector2::new(
        (x_min + x_extent / 2.) as f32,
        -(y_min + y_extent / 2.) as f32,
    );

    Camera2D {
        target: position,
        offset: RaylibVector2::new(W as f32 / 2., H as f32 / 2.),
        rotation: 0.,
        zoom: (zoom * 0.9),
    }
}

// Quick tryout of Raylib for setting up the mechanical structure
fn main() {
    let Args {
        file_path,
        maximum_iterations,
        error_constraint,
    } = Args::parse();
    let json_blob: String =
        fs::read_to_string(file_path).expect("Should have been able to read the file");

    let beam_sim: BeamSimulation = serde_json::from_str(&json_blob).unwrap();
    let solution = beam_sim.simulate(maximum_iterations, error_constraint);
    println!("Displacements: {solution:?}");

    let structure = beam_sim.structure;

    let displaced_points: Vec<_> = solution
        .chunks_exact(2)
        .zip(structure.points.iter())
        .map(|(values, &vector)| vector + Vector2::new(values[0], values[1]))
        .collect();

    let (mut rl, thread) = raylib::init().size(W, H).title("").build();

    let camera = get_camera(displaced_points.iter().chain(structure.points.iter()));
    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        d.clear_background(BACKGROUND);

        let mut mode = d.begin_mode2D(camera);
        for &(a, b) in &structure.connections {
            mode.draw_line_v(
                to_display(&structure.points[a]),
                to_display(&structure.points[b]),
                EDGE_COLOR.lerp(BACKGROUND, 0.75),
            );
        }

        for (n, v) in structure.points.iter().enumerate() {
            if structure.dbc.contains_key(&n) {
                mode.draw_circle_v(
                    to_display(v),
                    RADIUS / camera.zoom,
                    STABLE_NODE_COLOR.lerp(BACKGROUND, 0.75),
                );
            } else {
                mode.draw_circle_v(
                    to_display(v),
                    RADIUS / camera.zoom,
                    NODE_COLOR.lerp(BACKGROUND, 0.75),
                );
            }
        }

        for &(a, b) in &structure.connections {
            mode.draw_line_v(
                to_display(&displaced_points[a]),
                to_display(&displaced_points[b]),
                EDGE_COLOR,
            );
        }

        for (n, v) in displaced_points.iter().enumerate() {
            if structure.dbc.contains_key(&n) {
                mode.draw_circle_v(to_display(v), RADIUS / camera.zoom, STABLE_NODE_COLOR);
            } else {
                mode.draw_circle_v(to_display(v), RADIUS / camera.zoom, NODE_COLOR);
            }
        }
    }
}
