use core::f64;
use std::collections::HashMap;

use serde::{Deserialize, Serialize};

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
use crate::{solver::StiffnessMatrix, vector2::Vector2};
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

#[derive(Debug, Serialize, Deserialize)]
pub enum Material {
    Diamond,
    Wood,
    Pet,
    Lpde,
    HighStrengthConcrete,
    /// Custom material used for testing. Value is given in GPa
    Custom(f64),
}

impl Material {
    pub fn young_modulus(&self) -> f64 {
        (match self {
            Self::Diamond => 1050.0,
            Self::Wood => 9.5,
            Self::Pet => f64::consts::PI,
            Self::HighStrengthConcrete => 30.0,
            Self::Lpde => 0.228,
            Self::Custom(ym) => *ym,
        }) * 1E9
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct BeamStructure {
    pub points: Vec<Vector2>,
    pub connections: Vec<(usize, usize)>,
    pub dbc: HashMap<usize, (bool, bool)>, // drichlet boundary conditions
    pub material: Material,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct BeamSimulation {
    pub structure: BeamStructure,
    #[serde(with = "force_array_as_map")]
    pub forces: Vec<f64>,
}
impl BeamSimulation {
    pub fn simulate(&self, maximum_iterations: usize, error: f64) -> Vec<f64> {
        // Set the forces equal to the boundary conditions
        let mut f = self.forces.clone();
        for i in self.structure.dbc.keys() {
            if let Some((u1, u2)) = self.structure.dbc.get(i) {
                if *u1 {
                    f[2 * i] = 0.;
                }
                if *u2 {
                    f[2 * i + 1] = 0.;
                }
            }
        }
        // Set up StiffnessMatrix form beam structure
        let stiffness_mat: StiffnessMatrix = (&self.structure).into();
        // set up problem vector
        let mut u: Vec<f64> = vec![1.0; stiffness_mat.dim().0];
        // solve for u using gauss seidel
        stiffness_mat.solve_gauss_seidel(&f[..], &mut u, maximum_iterations, error);
        u
    }
}

mod force_array_as_map {
    use std::{collections::HashMap, fmt};

    use serde::{
        Deserializer, Serializer,
        de::{self, SeqAccess, Visitor},
        ser::SerializeTuple,
    };

    pub fn serialize<S>(vec: &[f64], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut map = HashMap::new();
        for (n, f) in vec
            .chunks_exact(2)
            .enumerate()
            .filter(|&(_, f)| f[0] != 0. || f[1] != 0.)
        {
            map.insert(n, (f[0], f[1]));
        }

        let mut tuple = serializer.serialize_tuple(2)?;
        tuple.serialize_element(&(vec.len() * 2))?;
        tuple.serialize_element(&map)?;
        tuple.end()
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<f64>, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct SparseVecVisitor;

        impl<'de> Visitor<'de> for SparseVecVisitor {
            type Value = Vec<f64>;

            fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
                f.write_str("a tuple [len, {index: value, ...}]")
            }

            fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
            where
                A: SeqAccess<'de>,
            {
                let len: usize = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(0, &self))?;

                let map: HashMap<usize, (f64, f64)> = seq
                    .next_element()?
                    .ok_or_else(|| de::Error::invalid_length(1, &self))?;

                let mut vec = vec![0.; len * 2];
                for (i, v) in map {
                    if i < len {
                        vec[2 * i] = v.0;
                        vec[2 * i + 1] = v.1;
                    }
                }

                Ok(vec)
            }
        }

        deserializer.deserialize_tuple(2, SparseVecVisitor)
    }
}
