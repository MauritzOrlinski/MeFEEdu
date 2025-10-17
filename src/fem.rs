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
use crate::solver::StiffnessMatrix;
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
    pub dirichlet_points: HashSet<usize>,
    pub material: Material,
}

pub fn fe(
    beam_struct: BeamStructure,
    forces: &[f64],
    maximum_iterations: usize,
    error: f64,
) -> Vec<f64> {
    let stiffness_mat: StiffnessMatrix = beam_struct.into();
    let mut u: Vec<f64> = vec![0.0; stiffness_mat.dim().0];
    stiffness_mat.solve_gauss_seidel(forces, &mut u, maximum_iterations, error);
    u
}
