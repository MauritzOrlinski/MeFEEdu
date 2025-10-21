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
use serde::{Deserialize, Serialize};
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

fn construct_ffi_vec(v: &vector2::Vector2) -> raylib::ffi::Vector2 {
    raylib::ffi::Vector2 {
        x: v.x as f32,
        y: v.y as f32,
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
    println!("Displacements: {:?}", solution);

    let structure = beam_sim.structure;
    let displaced_points: Vec<Vector2> = (0..structure.points.len())
        .map(|n| {
            (&Vector2 {
                x: solution[2 * n],
                y: solution[2 * n + 1],
            } + &structure.points[n])
                * -1.0
        })
        .collect();

    let (mut rl, thread) = raylib::init().size(W, H).title("").build();
    let mut camera = Camera2D {
        offset: RaylibVector2::new((W / 2) as f32, (H / 2) as f32),
        target: RaylibVector2::new(0.0, 0.0),
        rotation: 0.0,
        zoom: 1.0,
    };
    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        d.clear_background(BACKGROUND);
        let mut mode = d.begin_mode2D(&camera);
        for &(a, b) in &structure.connections {
            mode.draw_line_v(
                construct_ffi_vec(&displaced_points[a]),
                construct_ffi_vec(&displaced_points[b]),
                EDGE_COLOR,
            );
        }
        for v in &displaced_points {
            mode.draw_circle_v(construct_ffi_vec(v), RADIUS, NODE_COLOR);
        }

        for &v in structure.dbc.keys() {
            mode.draw_circle_v(
                construct_ffi_vec(&displaced_points[v]),
                RADIUS,
                STABLE_NODE_COLOR,
            );
        }
    }
}
