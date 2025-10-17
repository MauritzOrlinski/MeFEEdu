mod solver;
use raylib::prelude::*;

use crate::solver::StiffnessMatrix;

const RADIUS: f32 = 5.0;
const BACKGROUND: Color = Color::BLACK;
const NODE_COLOR: Color = Color::BLUE;
const EDGE_COLOR: Color = Color::WHITE;
const STABEE_NODE_COLOR: Color = Color::RED;
#[allow(dead_code)]
fn draw_polygon_filled(d: &mut RaylibDrawHandle, points: &[Vector2], color: Color) {
    if points.len() < 3 {
        return;
    }

    let center = points.iter().fold(Vector2::zero(), |acc, p| acc + *p) / (points.len() as f32);

    for i in 0..points.len() {
        let p1 = points[i];
        let p2 = points[(i + 1) % points.len()];
        d.draw_triangle(center, p1, p2, color);
    }
}

fn clicked_on_point(points: &Vec<Vector2>, pos: Vector2) -> bool {
    for v in points {
        if v.distance_to(pos) < RADIUS {
            return true;
        }
    }
    false
}

// Quick tryout of Raylib for setting up the mechanical structure
fn main() {
    let (mut rl, thread) = raylib::init().size(640, 480).title("Hello, World").build();
    let mut points: Vec<Vector2> = vec![];
    let mut stable: Vec<Vector2> = vec![];
    let mut edges: Vec<(Vector2, Vector2)> = vec![];
    let mut draw_conection = false;
    let mut last_point: Vector2 = Vector2 { x: 0.0, y: 0.0 };
    while !rl.window_should_close() {
        let mouse_pos = rl.get_mouse_position();
        if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
            if clicked_on_point(&points, mouse_pos) && draw_conection {
                if mouse_pos == last_point {
                    stable.push(last_point);
                } else {
                    edges.push((last_point, mouse_pos));
                }
                draw_conection = false;
            } else if clicked_on_point(&points, mouse_pos) && !draw_conection {
                draw_conection = true;
                last_point = mouse_pos;
            } else if draw_conection {
                points.push(mouse_pos);
                draw_conection = false;
                edges.push((last_point, mouse_pos));
            } else {
                points.push(mouse_pos);
            }
        }
        let mut d = rl.begin_drawing(&thread);

        d.clear_background(BACKGROUND);
        for (a, b) in &edges {
            d.draw_line_v(a, b, EDGE_COLOR);
        }
        for v in &points {
            d.draw_circle_v(v, RADIUS, NODE_COLOR);
        }

        for v in &stable {
            d.draw_circle_v(v, RADIUS, STABEE_NODE_COLOR);
        }
    }
}
