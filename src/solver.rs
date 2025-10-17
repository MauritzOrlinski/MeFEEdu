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
use sprs::CsMat;

pub struct StiffnessMatrix {
    matrix: CsMat<f32>,
}

impl StiffnessMatrix {
    pub fn from_csmat(mat: CsMat<f32>) -> Self {
        StiffnessMatrix { matrix: mat }
    }

    pub fn setup_stiffness_matrix() -> Self {
        !unimplemented!("To Do")
    }

    /**
     * Solves/Approximates Problems of type a = M x with error < eps (or maximal number of
     * iteration steps)
     */
    pub fn solve_gauss_seidel(
        &self,
        b: &[f32],
        x: &mut [f32],
        maximal_iteration: usize,
        eps: f32,
    ) -> Option<usize> {
        // compute Gauss Seidel: https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method
        for iter in 0..maximal_iteration {
            let mut a_ii: f32 = 0.0;
            for (i, row) in self.matrix.outer_iterator().enumerate() {
                let mut sigma: f32 = 0.0;
                for (j, &v) in row.iter() {
                    if i != j {
                        sigma += v * x[j];
                    } else {
                        a_ii = v;
                    }
                }
                let temp = x[i];
                x[i] = (b[i] - sigma) / a_ii;
                if (x[i] - temp).abs() < eps {
                    return Some(iter);
                }
            }
        }
        Some(maximal_iteration)
    }
}
