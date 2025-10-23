mod drawing_utils;
mod fem;
mod solver;
mod vector2;

use crate::drawing_utils::*;
use crate::fem::BeamSimulation;
use clap::Parser;
use raylib::prelude::Color;
use raylib::prelude::RaylibDraw;
use raylib::prelude::RaylibMode2DExt;
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
const W: i32 = 1000;

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

    let (mut rl, thread) = raylib::init()
        .size(W, H)
        .title("MeFEedu")
        .resizable()
        .build();

    let structure_extents =
        get_point_cloud_extents(displaced_points.iter().chain(structure.points.iter()));

    let node_color = |n| {
        if structure.dbc.contains_key(&n) {
            FIXED_NODE_COLOR
        } else {
            NODE_COLOR
        }
    };

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        d.clear_background(BACKGROUND);

        let camera = get_camera(
            structure_extents,
            d.get_screen_width(),
            d.get_screen_height(),
        );

        let mut mode = d.begin_mode2D(camera);

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
                &structure.dbc,
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
