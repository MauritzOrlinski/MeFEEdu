use crate::vector2::Vector2;
use raylib::camera;
use raylib::ffi::true_;
use raylib::prelude::Camera2D;
use raylib::prelude::RaylibDraw;
use raylib::prelude::RaylibMode2DExt;
use raylib::prelude::Vector2 as RaylibVector2;
use raylib::{prelude::Color, prelude::RaylibDrawHandle};
use std::collections::HashMap;
use std::fs;
use std::ops::Mul;

const RADIUS: f32 = 5.0;
const TRIANGLE_SIZE: f32 = 10.0;
const TRIANGLE_COLOR: Color = Color::WHITE;
const PULLEY_WHEELS_RADIUS: f32 = 0.4;

pub const BACKGROUND: Color = Color::BLACK;
pub const NODE_COLOR: Color = Color::BLUE;
pub const EDGE_COLOR: Color = Color::WHITE;
pub const FIXED_NODE_COLOR: Color = Color::RED;

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

fn to_display_vector(v: &Vector2) -> raylib::ffi::Vector2 {
    raylib::ffi::Vector2 {
        x: v.x as f32,
        y: -v.y as f32,
    }
}

pub fn get_camera(
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

// fn draw_fixed_point(
//     d: &mut RaylibDrawHandle,
//     camera: &Camera2D,
//     point: &Vector2,
//     dof_locks: (bool, bool),
// ) {
//     let triangle_size = TRIANGLE_SIZE / camera.zoom;
//     match dof_locks {
//         (true, true) => d.draw_triangle_lines(
//             to_display_vector(point),
//             to_display_vector(&(*point + Vector2 { x: -0.8, y: -1. }.mul(triangle_size as f64))),
//             to_display_vector(&(*point - Vector2 { x: 0.8, y: -1. }.mul(triangle_size as f64))),
//             TRIANGLE_COLOR,
//         ),
//         (true, _) => {
//             let v2 = *point + Vector2 { x: -1., y: -0.8 }.mul(triangle_size as f64);
//             let v3 = *point + Vector2 { x: -1., y: 0.8 }.mul(triangle_size as f64);
//             d.draw_triangle_lines(
//                 to_display_vector(point),
//                 to_display_vector(&v2),
//                 to_display_vector(&v3),
//                 TRIANGLE_COLOR,
//             );
//             d.draw_circle_v(
//                 to_display_vector(
//                     &(v2 + Vector2 {
//                         x: -PULLEY_WHEELS_RADIUS as f64,
//                         y: 0.01 * TRIANGLE_SIZE as f64,
//                     }),
//                 ),
//                 PULLEY_WHEELS_RADIUS,
//                 TRIANGLE_COLOR,
//             );
//             d.draw_circle_v(
//                 to_display_vector(
//                     &(v3 + Vector2 {
//                         x: -PULLEY_WHEELS_RADIUS as f64,
//                         y: -0.01 * TRIANGLE_SIZE as f64,
//                     }),
//                 ),
//                 PULLEY_WHEELS_RADIUS,
//                 TRIANGLE_COLOR,
//             );
//         }
//         (_, true) => {
//             let v2 = *point + Vector2 { x: -0.8, y: -1. }.mul(triangle_size as f64);
//             let v3 = *point + Vector2 { x: 0.8, y: -1. }.mul(triangle_size as f64);
//             d.draw_triangle_lines(
//                 to_display_vector(point),
//                 to_display_vector(&v2),
//                 to_display_vector(&v3),
//                 TRIANGLE_COLOR,
//             );
//             d.draw_circle_v(
//                 to_display_vector(
//                     &(v2 + Vector2 {
//                         x: 0.07 * TRIANGLE_SIZE as f64,
//                         y: -PULLEY_WHEELS_RADIUS as f64,
//                     }),
//                 ),
//                 PULLEY_WHEELS_RADIUS,
//                 TRIANGLE_COLOR,
//             );
//             d.draw_circle_v(
//                 to_display_vector(
//                     &(v3 + Vector2 {
//                         x: -0.07 * TRIANGLE_SIZE as f64,
//                         y: -PULLEY_WHEELS_RADIUS as f64,
//                     }),
//                 ),
//                 PULLEY_WHEELS_RADIUS,
//                 TRIANGLE_COLOR,
//             );
//         }
//         _ => (),
//     }
// }

pub fn draw_structure_fade<I, J>(
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
        d.draw_line_ex(
            to_display_vector(&points[a]),
            to_display_vector(&points[b]),
            0.5,
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

pub fn draw_structure<I, J>(
    d: &mut RaylibDrawHandle,
    points: &[Vector2],
    connections: &[(usize, usize)],
    dbc: &HashMap<usize, (bool, bool)>,
    camera: &Camera2D,
    node_color: I,
    edge_color: J,
) where
    I: Fn(usize) -> Color,
    J: Fn(usize) -> Color,
{
    draw_structure_fade(d, points, connections, camera, node_color, edge_color, 0.);
    // for (&i, &lock) in dbc.iter() {
    //     draw_fixed_point(d, camera, &points[i], lock);
    // }
}
