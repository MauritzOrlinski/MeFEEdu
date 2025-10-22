mod fem;
mod solver;
mod vector2;

use crate::fem::BeamSimulation;
use crate::fem::BeamStructure;
use clap::Parser;
use raylib::camera;
use raylib::prelude::Camera2D;
use raylib::prelude::RaylibDraw;
use raylib::prelude::RaylibMode2DExt;
use raylib::prelude::Vector2 as RaylibVector2;
use raylib::{prelude::Color, prelude::RaylibDrawHandle};
use std::collections::HashMap;
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
    #[arg(short, long, default_value_t = false)]
    no_simulate: bool,
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

fn to_display_vector(v: &vector2::Vector2) -> raylib::ffi::Vector2 {
    raylib::ffi::Vector2 {
        x: v.x as f32,
        y: -v.y as f32,
    }
}

fn get_point_cloud_extents<'a, I>(mut points: I) -> (f64, f64, f64, f64)
where
    I: Iterator<Item = &'a Vector2>,
{
    let first = points
        .next()
        .expect("Point cloud must at least contain one point");

    points.fold(
        (first.x, first.x, first.y, first.y),
        |(x_min, x_max, y_min, y_max), point| {
            (
                x_min.min(point.x),
                x_max.max(point.x),
                y_min.min(point.y),
                y_max.max(point.y),
            )
        },
    )
}

fn get_camera(
    (x_min, x_max, y_min, y_max): (f64, f64, f64, f64),
    width: i32,
    height: i32,
) -> Camera2D {
    let x_extent = x_max - x_min;

    let y_extent = y_max - y_min;
    let max_x_zoom = (width as f64) / x_extent;
    let max_y_zoom = (height as f64) / y_extent;

    let zoom = max_y_zoom.min(max_x_zoom) as f32;
    let position = RaylibVector2::new(
        (x_min + x_extent / 2.) as f32,
        -(y_min + y_extent / 2.) as f32,
    );

    Camera2D {
        target: position,
        offset: RaylibVector2::new(width as f32 / 2., height as f32 / 2.),
        rotation: 0.,
        zoom: (zoom * 0.9),
    }
}

fn draw_structure_fade<I, J>(
    d: &mut RaylibDrawHandle,
    points: &[Vector2],
    connections: &[(usize, usize)],
    camera: &Camera2D,
    node_color: I,
    edge_color: J,
    fade: f32,
) where
    I: Fn(usize) -> Color,
    J: Fn(usize) -> Color,
{
    for (n, &(a, b)) in connections.iter().enumerate() {
        d.draw_line_v(
            to_display_vector(&points[a]),
            to_display_vector(&points[b]),
            edge_color(n).lerp(BACKGROUND, fade),
        );
    }

    for (n, v) in points.iter().enumerate() {
        d.draw_circle_v(
            to_display_vector(v),
            RADIUS / camera.zoom,
            node_color(n).lerp(BACKGROUND, fade),
        );
    }
}

fn draw_structure<I, J>(
    d: &mut RaylibDrawHandle,
    points: &[Vector2],
    connections: &[(usize, usize)],
    camera: &Camera2D,
    node_color: I,
    edge_color: J,
) where
    I: Fn(usize) -> Color,
    J: Fn(usize) -> Color,
{
    draw_structure_fade(d, points, connections, camera, node_color, edge_color, 0.);
}

// Quick tryout of Raylib for setting up the mechanical structure
fn main() {
    let Args {
        file_path,
        maximum_iterations,
        error_constraint,
        no_simulate,
    } = Args::parse();
    let json_blob: String =
        fs::read_to_string(file_path).expect("Should have been able to read the file");

    let beam_sim: BeamSimulation = serde_json::from_str(&json_blob).unwrap();
    let solution = if !no_simulate {
        beam_sim.simulate(maximum_iterations, error_constraint)
    } else {
        vec![]
    };
    println!("Displacements: {solution:?}");

    let structure = beam_sim.structure;

    let displaced_points: Vec<_> = solution
        .chunks_exact(2)
        .zip(structure.points.iter())
        .map(|(values, &vector)| vector + Vector2::new(values[0], values[1]))
        .collect();

    let (mut rl, thread) = raylib::init().size(W, H).title("").resizable().build();

    let structure_extents =
        get_point_cloud_extents(displaced_points.iter().chain(structure.points.iter()));

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        d.clear_background(BACKGROUND);

        let camera = get_camera(
            structure_extents,
            d.get_screen_width(),
            d.get_screen_height(),
        );

        let mut mode = d.begin_mode2D(camera);

        let node_color = |n| {
            if structure.dbc.contains_key(&n) {
                STABLE_NODE_COLOR
            } else {
                NODE_COLOR
            }
        };

        draw_structure_fade(
            &mut mode,
            &structure.points,
            &structure.connections,
            &camera,
            node_color,
            |_| EDGE_COLOR,
            if no_simulate { 0. } else { 0.66 },
        );

        if !no_simulate {
            draw_structure(
                &mut mode,
                &displaced_points,
                &structure.connections,
                &camera,
                node_color,
                |n| {
                    let (a, b) = structure.connections[n];
                    let original_length = structure.points[a].distance_to(structure.points[b]);
                    let displaced_length = displaced_points[a].distance_to(displaced_points[b]);

                    let diff = (original_length - displaced_length).abs();
                    let ratio = diff / original_length;
                    EDGE_COLOR.lerp(Color::RED, 50. * ratio.min(1.) as f32)
                },
            );
        }
    }
}
