use std::collections::HashMap;

use crate::fem::BeamStructure;
use raylib::math::Vector2;
use sprs::{CsMat, CsVecViewMut};
/**
* Implementing the FEM for 2d mechanincal Problems using Triangular Discretization
*
* Basic Idea:
* f = K u
* where:
* - f is the vector of forces \in R^n. f is given by the user setting forces that effect specific
*   nodes
* - K is the Stiffness matrix in \in R^(n x n). K can be computed by the spring equations and
*   stiffness constants for the given Triangular Discretization
* - u is the displacement vector \in R^n. It will be solved for
*
* Due to the big size of the Matrix a Gauss Seidel method is used instead of solving the full LSE
*
* For the matrix a CSR representation is choosen
*/

pub const DOG_PER_NODE: usize = 2;

pub struct StiffnessMatrix {
    matrix: CsMat<f64>,
}

fn rot_matrix(theta: f32) -> CsMat<f64> {
    let c = f32::cos(theta) as f64;
    let s = f32::sin(theta) as f64;

    let c2 = c * c;
    let s2 = s * s;
    let cs = c * s;

    let matrix = ndarray::arr2(&[[c2, cs], [cs, s2]]);

    CsMat::csr_from_dense((&matrix).into(), 0.0)
}

fn signed_angle_2d(a: Vector2, b: Vector2) -> f32 {
    // https://wumbo.net/formulas/angle-between-two-vectors-2d/
    // since we just want (1,0) we can simplify a little
    let vec = a - b;
    f32::atan2(-vec.y, vec.x)
}

fn place_matrix(source: &CsMat<f64>, target: &mut CsMat<f64>, row: usize, col: usize) {
    let (width, height) = source.shape();

    for i in row..row + width {
        for j in col..col + height {
            let old = *target.get(i, j).unwrap_or(&0.);
            target.set(i, j, old + *source.get(i, j).unwrap_or(&0.));
        }
    }
}

pub fn apply_dbc(matrix: &mut CsMat<f64>, dbc: &HashMap<usize, (Option<f64>, Option<f64>)>) {
    for &i in dbc.keys() {
        for j in 0..matrix.rows() {
            matrix.set(i, j, 0.0);
            matrix.set(j, i, 0.0);
        }
        matrix.set(i, i, 1.0);
    }
}

impl From<BeamStructure> for StiffnessMatrix {
    fn from(beam_structure: BeamStructure) -> Self {
        let cross = 1.;

        let mat_size = DOG_PER_NODE * beam_structure.points.len();

        let mut matrix: CsMat<f64> = CsMat::zero((mat_size, mat_size));

        for (a, b) in beam_structure.conections {
            let point_a = beam_structure.points[a];
            let point_b = beam_structure.points[b];
            let length = point_a.distance_to(point_b);

            let stiffness = beam_structure.material.young_modulus() * cross / length as f64;

            let beam_angle = signed_angle_2d(point_a, point_b);

            let mut local_matrix = rot_matrix(beam_angle);
            local_matrix *= stiffness;

            let mut inv_local_matrix = local_matrix.clone();
            inv_local_matrix *= -1.;

            place_matrix(
                &local_matrix,
                &mut matrix,
                a * DOG_PER_NODE,
                a * DOG_PER_NODE,
            );
            place_matrix(
                &local_matrix,
                &mut matrix,
                b * DOG_PER_NODE,
                b * DOG_PER_NODE,
            );

            place_matrix(
                &inv_local_matrix,
                &mut matrix,
                a * DOG_PER_NODE,
                b * DOG_PER_NODE,
            );
            place_matrix(
                &inv_local_matrix,
                &mut matrix,
                b * DOG_PER_NODE,
                a * DOG_PER_NODE,
            );
        }
        apply_dbc(&mut matrix, &beam_structure.dbc);
        Self { matrix }
    }
}

impl From<CsMat<f64>> for StiffnessMatrix {
    fn from(value: CsMat<f64>) -> Self {
        StiffnessMatrix { matrix: value }
    }
}

impl StiffnessMatrix {
    pub fn dim(&self) -> (usize, usize) {
        (self.matrix.outer_dims(), self.matrix.inner_dims())
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
            for (i, row) in self.matrix.outer_iterator().enumerate() {
                let mut sigma: f64 = 0.0;
                for (j, &v) in row.iter() {
                    if i != j {
                        sigma += v * x[j];
                    } else {
                        a_ii = v;
                    }
                }
                let temp = x[i];
                if a_ii == 0.0 {
                    return None;
                }
                x[i] = (b[i] - sigma) / a_ii;
                if (x[i] - temp).abs() < eps {
                    return Some(iter);
                }
            }
        }
        Some(maximal_iteration)
    }
}
