use core::f64;
use std::collections::{HashMap, HashSet};

/**
* Implementing the FEM for 2d mechanincal Problems using Triangular Discretization
* We need to
* Discretize:
* Translate the User input into a 2d Mesh made up from triangles
*
*
* Solve:
* f = K u
* where:
* - f is the vector of forces \in R^n. f is given by the user setting forces that effect specific
*   nodes
* - K is the Stiffness matrix in \in R^(n x n). K can be computed by the spring equations and
*   stiffness constants for the given Triangular Discretization
* - u is the displacement vector \in R^n. It will be solved for
*/
use crate::solver::{self, DOG_PER_NODE, StiffnessMatrix};
use raylib::prelude::Vector2;
// struct Triangle(Vector2, Vector2, Vector2);
//
// pub struct TriangularMesh {
//     triangles: Vec<Triangle>,
// }
//
// impl TriangularMesh {
//     pub fn discretize() -> Self {
//         !unimplemented!("Todo")
//     }
// }

pub enum Material {
    Diamond,
    Wood,
    PET,
    LPDE,
    HighStrengthConcrete,
}

impl Material {
    pub fn young_modulus(&self) -> f64 {
        (match self {
            Self::Diamond => 1050.0,
            Self::Wood => 9.5,
            Self::PET => f64::consts::PI,
            Self::HighStrengthConcrete => 30.0,
            Self::LPDE => 0.228,
        }) * 10E9
    }
}

pub struct BeamStructure {
    pub points: Vec<Vector2>,
    pub conections: Vec<(usize, usize)>,
    pub dbc: HashMap<usize, (Option<f64>, Option<f64>)>, // drichlet boundary conditions
    pub material: Material,
}

pub fn fe(
    beam_struct: BeamStructure,
    forces: &[f64],
    maximum_iterations: usize,
    error: f64,
) -> Vec<f64> {
    // Set the forces equal to the boundary conditions
    let mut f = forces.to_vec();
    for i in beam_struct.dbc.keys() {
        if let Some((u1, u2)) = beam_struct.dbc.get(i) {
            if let &Some(x) = u1 {
                f[2 * i] = x;
            }
            if let &Some(x) = u2 {
                f[2 * i + 1] = x;
            }
        }
    }
    // Set up StiffnessMatrix form beam structure
    let stiffness_mat: StiffnessMatrix = beam_struct.into();
    // set up problem vector
    let mut u: Vec<f64> = vec![1.0; stiffness_mat.dim().0];
    // solve for u using gauss seidel
    stiffness_mat.solve_gauss_seidel(&f[..], &mut u, maximum_iterations, error);
    u
}
