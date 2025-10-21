use super::*;
use std::{collections::HashMap, f64::consts::PI};

use ndarray::arr2;

use crate::{
    fem::{BeamStructure, Material},
    vector2::Vector2,
};

// error tolerance
const EPSILON: f64 = 1. / 1E6;

fn rough_compare<'a, I>(actual: I, reference: I)
where
    I: Iterator<Item = &'a f64>,
{
    actual
        .zip(reference)
        .for_each(|(&a, &b)| {
            let e = if a == 0. {
                if (b).abs() < EPSILON {
                    // both are 0, they are equal
                    return;
                } 
                panic!("{a} (actual) and {b} (reference) were too different to be considered equal")
            }
            else {
                ((a - b) / a).abs()
            };

            assert!(
                e < EPSILON,
                "{a} (actual) and {b} (reference) were too different to be considered equal (epsilon was {e})"
            )
        });
}

#[test]
fn gauss_seidel_solver_id() {
    let matrix = StiffnessMatrix {
        matrix: arr2(&[[1.0, 0.0], [0.0, 1.0]]),
    };
    let b = [2.0, 2.0];
    let x = &mut [1.0, 1.0];
    matrix.solve_gauss_seidel(&b, x, 200, EPSILON);
    rough_compare(b.iter(), x.iter());
}

#[test]
fn gauss_seidel_simple() {
    let matrix = StiffnessMatrix {
        matrix: arr2(&[[2., -1., 1.], [-1., 2., -1.], [1., -1., -2.]]),
    };
    let b = [-1., -1., 0.];
    let x = &mut [0., 0., 0.];
    matrix.solve_gauss_seidel(&b, x, 200, EPSILON);
    rough_compare(b.iter(), x.iter());
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

    let stiff_mat: StiffnessMatrix = (&beam_struct).into();

    println!("{stiff_mat:?}");

    let expected_result = arr2(&[
        [0.25, 0.433012701892, -0.25, -0.433012701892],
        [0.433012701892, 0.75, -0.433012701892, -0.75],
        [-0.25, -0.433012701892, 0.25, 0.433012701892],
        [-0.433012701892, -0.75, 0.433012701892, 0.75],
    ]) * 2E9;

    rough_compare(stiff_mat.matrix.iter(), expected_result.iter());
}

#[test]
fn test_stiffness_matrix_simple_truss() {
    let points: Vec<_> = ([(0., 0.), (0.5, (PI / 3.).sin()), (1., 0.)])
        .iter()
        .map(|(x, y)| Vector2::new(*x, *y))
        .collect();

    let connections = vec![(0, 1), (0, 2), (1, 2)];
    let mut dbc: HashMap<usize, (bool, bool)> = HashMap::new();
    dbc.insert(0, (true, true));
    dbc.insert(2, (false, true));

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

    let stiff_mat: StiffnessMatrix = (&beam_struct).into();

    println!("{stiff_mat:?}");

    let expected_result = arr2(&[
        [1.25, 0.433012701892, -0.25, -0.433012701892, -1., 0.],
        [0.433012701892, 0.75, -0.433012701892, -0.75, 0., 0.],
        [-0.25, -0.433012701892, 0.5, 0., -0.25, 0.433012701892],
        [-0.433012701892, -0.75, 0., 1.5, 0.433012701892, -0.75],
        [-1., 0., -0.25, 0.433012701892, 1.25, -0.433012701892],
        [0., 0., 0.433012701892, -0.75, -0.433012701892, 0.75],
    ]) * 2E9;

    rough_compare(stiff_mat.matrix.iter(), expected_result.iter());
}
