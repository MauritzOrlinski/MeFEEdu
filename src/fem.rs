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
struct Triangle(Vector2, Vector2, Vector2);

pub struct TriangularMesh {
    triangles: Vec<Triangle>,
}

impl TriangularMesh {
    pub fn discretize() -> Self {
        !unimplemented!("Todo")
    }
}
