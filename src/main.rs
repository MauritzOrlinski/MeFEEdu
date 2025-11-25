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

    let mut beam_sim: BeamSimulation = serde_json::from_str(&json_blob).unwrap();

    let (mut rl, thread) = raylib::init()
        .size(W, H)
        .title("MeFEedu")
        .resizable()
        .build();

    let mut drag_start = None;

    beam_sim.forces = beam_sim.forces.iter().map(|_| 0.).collect();
    let forces = beam_sim.forces.clone();

    while !rl.window_should_close() {
        let mut d = rl.begin_drawing(&thread);

        beam_sim.forces = forces
            .chunks_exact(2)
            .enumerate()
            .map(|(i, v)| {
                let v = Vector2::new(v[0], v[1]);
                if let Some((start_i, _, delta)) = drag_start
                    && i == start_i
                {
                    let mut delta: Vector2 = delta;
                    delta.x *= -1.;
                    delta = delta * 5000.;

                    v + delta
                } else {
                    v
                }
            })
            .flat_map(|e| [e.x, e.y])
            .collect();

        let solution = if !no_simulate {
            beam_sim.simulate(maximum_iterations, error_constraint)
        } else {
            vec![]
        };

        let structure = &beam_sim.structure;

        let displaced_points: Vec<_> = solution
            .chunks_exact(2)
            .zip(structure.points.iter())
            .map(|(values, &vector)| vector + Vector2::new(values[0], values[1]))
            .collect();

        let structure_extents =
            get_point_cloud_extents(displaced_points.iter().chain(structure.points.iter()));

        let node_color = |n| {
            if structure.dbc.contains_key(&n) {
                FIXED_NODE_COLOR
            } else {
                NODE_COLOR
            }
        };

        d.clear_background(BACKGROUND);

        let camera = get_camera(
            structure_extents,
            d.get_screen_width(),
            d.get_screen_height(),
        );

        if d.is_mouse_button_down(raylib::ffi::MouseButton::MOUSE_BUTTON_LEFT)
            && drag_start.is_none()
        {
            let mut mouse_pos = d.get_screen_to_world2D(d.get_mouse_position(), camera);
            mouse_pos.y *= -1.;

            for (i, node) in displaced_points.iter().enumerate() {
                if node.distance_to(mouse_pos.into()) < 10. {
                    drag_start = Some((i, *node, Vector2::new(0., 0.)));
                    break;
                }
            }
        }

        if let Some((start_i, start_pos, _)) = drag_start
            && d.is_mouse_button_up(raylib::ffi::MouseButton::MOUSE_BUTTON_LEFT)
        {
            let mouse_pos = d.get_screen_to_world2D(d.get_mouse_position(), camera);
            println!("Drag from {start_i} to {mouse_pos:?}");

            let delta = start_pos - mouse_pos.into();

            beam_sim.forces[2 * start_i] = -delta.x * 5000.;
            beam_sim.forces[2 * start_i + 1] = delta.y * 5000.;

            drag_start = None;
        }

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

        drag_start = drag_start.map(|(i, start, _)| {
            let mut mouse_position = mode.get_screen_to_world2D(mode.get_mouse_position(), camera);
            mouse_position.y *= -1.;
            let mut delta = start - mouse_position.into();

            delta.y *= -1.;

            (i, start, delta)
        });

        println!("{drag_start:?}");
    }
}
