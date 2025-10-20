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

fn signed_angle_2d(a: &Vector2, b: &Vector2) -> f64 {
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

impl From<BeamStructure> for StiffnessMatrix {
    fn from(beam_structure: BeamStructure) -> Self {
        let cross = 1.;

        let mat_size = DOG_PER_NODE * beam_structure.points.len();

        let mut matrix = Array2::<_>::zeros((mat_size, mat_size));

        for (a, b) in beam_structure.connections {
            let point_a = &beam_structure.points[a];
            let point_b = &beam_structure.points[b];
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

#[cfg(test)]
mod test_stiffness_matrix {
    use std::{collections::HashMap, f64::consts::PI};

    use ndarray::arr2;

    use crate::{
        fem::{BeamStructure, Material},
        solver::{Matrix, StiffnessMatrix, add_region, apply_dbc},
        vector2::Vector2,
    };

    // error tolerance
    const EPSILON: f64 = 1. / 10E5;

    fn rough_compare_matrices(actual: &Matrix, reference: &Matrix) {
        actual.iter()
            .zip(reference.iter())
            .for_each(|(a, b)| {
                let e = if *a == 0. {
                    if *b == 0. {
                        // both are 0, they are equal
                        return;
                    }
                    // b is nonzero, use it for normalizing instead
                    ((*a - *b) / *b).abs()
                }
                else {
                    ((*a - *b) / *a).abs()
                };

                assert!(
                    e < EPSILON,
                    "{a} (actual) and {b} (reference) were too different to be considered equal (epsilon was {e})"
                )
            });
    }

    #[test]
    fn test_add_region() {
        const SIZE: usize = 5;
        const SIZE2: usize = 3;
        let mut target = ndarray::arr2(&[[1.; SIZE]; SIZE]);
        let src = ndarray::arr2(&[[2.; SIZE2]; SIZE2]);

        add_region(&src, &mut target, 1, 1);

        println!("{target}");

        assert!(
            target
                == ndarray::arr2(&[
                    [1., 1., 1., 1., 1.],
                    [1., 3., 3., 3., 1.],
                    [1., 3., 3., 3., 1.],
                    [1., 3., 3., 3., 1.],
                    [1., 1., 1., 1., 1.]
                ])
        )
    }

    #[test]
    fn test_apply_dbc() {
        // we will have 3 nodes
        const SIZE: usize = 6;

        let mut dbc = HashMap::new();
        // lock both x and y here
        dbc.insert(0, (true, true));

        // only lock y
        dbc.insert(2, (false, true));

        let mut matrix = arr2(&[[5.; SIZE]; SIZE]);

        apply_dbc(&mut matrix, &dbc);

        // we lock x and y on index 0, so rows/cols 0*2 and 0*2+1 are 0, diagonal entries are 1
        // we lock y on index 2, so row/col 2*2+1=5 is 0, diagonal entry is 1
        let expected_result = arr2(&[
            [1., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
            [0., 0., 5., 5., 5., 0.],
            [0., 0., 5., 5., 5., 0.],
            [0., 0., 5., 5., 5., 0.],
            [0., 0., 0., 0., 0., 1.],
        ]);

        println!("{matrix}");
        assert!(expected_result == matrix);
    }

    #[test]
    fn test_stiffness_matrix_single_beam() {
        let points: Vec<_> = ([(0., 0.), (0.5, (PI / 3.).sin())])
            .iter()
            .map(|(x, y)| Vector2::new(*x, *y))
            .collect();

        let connections = vec![(0, 1)];
        let dbc: HashMap<usize, (bool, bool)> = HashMap::new();

        // values taken from https://www.youtube.com/watch?v=9bnFVE88PaM
        // we don't support setting the cross section yet, so pretend the material is much weaker
        // than it actually is and assume a 1m^2 cross section
        let material = Material::Custom(200. * 0.01);

        let beam_struct = BeamStructure {
            points,
            connections,
            dbc,
            material,
        };

        println!("{beam_struct:?}");

        let stiff_mat: StiffnessMatrix = beam_struct.into();

        println!("{stiff_mat:?}");

        let expected_result = arr2(&[
            [0.25, 0.433012701892, -0.25, -0.433012701892],
            [0.433012701892, 0.75, -0.433012701892, -0.75],
            [-0.25, -0.433012701892, 0.25, 0.433012701892],
            [-0.433012701892, -0.75, 0.433012701892, 0.75],
        ]) * 2.
            * 10E8;

        rough_compare_matrices(&stiff_mat.matrix, &expected_result);
    }

    #[test]
    fn test_stiffness_matrix_simple_truss() {
        let points: Vec<_> = ([(0., 0.), (0.5, (PI / 3.).sin()), (1., 0.)])
            .iter()
            .map(|(x, y)| Vector2::new(*x, *y))
            .collect();

        let connections = vec![(0, 1), (0, 2), (1, 2)];
        let dbc: HashMap<usize, (bool, bool)> = HashMap::new();

        // values taken from https://www.youtube.com/watch?v=9bnFVE88PaM
        // we don't support setting the cross section yet, so pretend the material is much weaker
        // than it actually is and assume a 1m^2 cross section
        let material = Material::Custom(200. * 0.01);

        let beam_struct = BeamStructure {
            points,
            connections,
            dbc,
            material,
        };

        println!("{beam_struct:?}");

        let stiff_mat: StiffnessMatrix = beam_struct.into();

        println!("{stiff_mat:?}");

        let expected_result = arr2(&[
            [1.25, 0.433012701892, -0.25, -0.433012701892, -1., 0.],
            [0.433012701892, 0.75, -0.433012701892, -0.75, 0., 0.],
            [-0.25, -0.433012701892, 0.5, 0., -0.25, 0.433012701892],
            [-0.433012701892, -0.75, 0., 1.5, 0.433012701892, -0.75],
            [-1., 0., -0.25, 0.433012701892, 1.25, -0.433012701892],
            [0., 0., 0.433012701892, -0.75, -0.433012701892, 0.75],
        ]) * 2.
            * 10E8;

        rough_compare_matrices(&stiff_mat.matrix, &expected_result);
    }
}
