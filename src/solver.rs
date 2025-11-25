use std::collections::HashMap;

use crate::{fem::BeamStructure, vector2::Vector2};
use ndarray::{Array2, s};
// * Implementing the FEM for 2d mechanincal Problems using Triangular Discretization
// *
// * Basic Idea:
// * f = K u
// * where:
// * - f is the vector of forces \in R^n. f is given by the user setting forces that effect specific
// *   nodes
// * - K is the Stiffness matrix in \in R^(n x n). K can be computed by the spring equations and
// *   stiffness constants for the given Triangular Discretization
// * - u is the displacement vector \in R^n. It will be solved for
// *
// * Due to the big size of the Matrix a Gauss Seidel method is used instead of solving the full LSE
// *
// * For the matrix a CSR representation is choosen

pub const DOG_PER_NODE: usize = 2;

type Matrix = Array2<f64>;

#[derive(Debug)]
pub struct StiffnessMatrix {
    matrix: Matrix,
}

fn rot_matrix(theta: f64) -> Matrix {
    let c = f64::cos(theta);
    let s = f64::sin(theta);

    let c2 = c * c;
    let s2 = s * s;
    let cs = c * s;

    ndarray::arr2(&[[c2, cs], [cs, s2]])
}

fn signed_angle_2d(a: Vector2, b: Vector2) -> f64 {
    // https://wumbo.net/formulas/angle-between-two-vectors-2d/
    // since we just want (1,0) we can simplify a little
    let vec = b - a;
    f64::atan2(vec.y, vec.x)
}

fn add_region(source: &Matrix, target: &mut Matrix, row: usize, col: usize) {
    let (height, width) = source.dim();

    target
        .slice_mut(s![row..row + height, col..col + width])
        .iter_mut()
        .zip(source)
        .for_each(|(target, src)| *target += *src);
}

fn apply_constraint(matrix: &mut Matrix, index: usize) {
    matrix.row_mut(index).fill(0.);
    matrix.column_mut(index).fill(0.);

    matrix[[index, index]] = 1.;
}

pub fn apply_dbc(matrix: &mut Matrix, dbc: &HashMap<usize, (bool, bool)>) {
    for (iter, (x_const, y_const)) in dbc.iter() {
        if *x_const {
            apply_constraint(matrix, *iter * 2);
        }

        if *y_const {
            apply_constraint(matrix, *iter * 2 + 1);
        }
    }
}
impl From<&BeamStructure> for StiffnessMatrix {
    fn from(beam_structure: &BeamStructure) -> Self {
        let cross = 1.;

        let mat_size = DOG_PER_NODE * beam_structure.points.len();

        let mut matrix = Array2::<_>::zeros((mat_size, mat_size));

        for &(a, b) in &beam_structure.connections {
            let point_a = beam_structure.points[a];
            let point_b = beam_structure.points[b];
            let length = point_a.distance_to(point_b);

            let stiffness = cross / length;

            let beam_angle = signed_angle_2d(point_a, point_b);

            let mut local_matrix = rot_matrix(beam_angle);

            local_matrix *= stiffness;

            let mut inv_local_matrix = local_matrix.clone();
            inv_local_matrix *= -1.;

            add_region(
                &local_matrix,
                &mut matrix,
                a * DOG_PER_NODE,
                a * DOG_PER_NODE,
            );
            add_region(
                &local_matrix,
                &mut matrix,
                b * DOG_PER_NODE,
                b * DOG_PER_NODE,
            );
            add_region(
                &inv_local_matrix,
                &mut matrix,
                a * DOG_PER_NODE,
                b * DOG_PER_NODE,
            );
            add_region(
                &inv_local_matrix,
                &mut matrix,
                b * DOG_PER_NODE,
                a * DOG_PER_NODE,
            );
        }

        apply_dbc(&mut matrix, &beam_structure.dbc);
        Self {
            matrix: matrix * beam_structure.material.young_modulus(),
        }
    }
}
impl From<Matrix> for StiffnessMatrix {
    fn from(value: Matrix) -> Self {
        StiffnessMatrix { matrix: value }
    }
}

impl StiffnessMatrix {
    pub fn dim(&self) -> (usize, usize) {
        self.matrix.dim()
    }

    /**
     * Solves/Approximates Problems of type a = M x with error < eps (or maximal number of
     * iteration steps)
     */
    pub fn solve_gauss_seidel(
        &self,
        b: &[f64],
        x: &mut [f64],
        maximal_iteration: usize,
        eps: f64,
    ) -> Option<usize> {
        // compute Gauss Seidel: https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
        for iter in 0..maximal_iteration {
            let mut a_ii: f64 = 0.0;
            let mut max_err: f64 = 0.0;
            for (i, row) in self.matrix.rows().into_iter().enumerate() {
                let mut sigma: f64 = 0.0;
                for (j, &v) in row.iter().enumerate() {
                    if i != j {
                        sigma += v * x[j];
                    } else {
                        a_ii = v;
                    }
                }
                let temp = x[i];
                if a_ii == 0.0 {
                    panic!("Diagonal contains Zero: Gauss Seidel not applicable")
                }
                x[i] = (b[i] - sigma) / a_ii;
                max_err = f64::max(max_err, (x[i] - temp).abs());
            }
            if max_err < eps {
                return Some(iter);
            }
        }
        Some(maximal_iteration)
    }
}

#[cfg(test)]
#[path = "./tests/solver.rs"]
mod tests_solver;
