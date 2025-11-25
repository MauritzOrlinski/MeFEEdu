use raylib::core::math::Vector2 as RaylibVec2;
use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};

#[derive(Debug, Clone, Copy)]
pub struct Vector2 {
    pub x: f64,
    pub y: f64,
}

impl<'de> Deserialize<'de> for Vector2 {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let arr = <[f64; 2]>::deserialize(deserializer)?;
        Ok(arr.into())
    }
}

impl Serialize for Vector2 {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        [self.x, self.y].serialize(serializer)
    }
}

impl From<RaylibVec2> for Vector2 {
    fn from(value: RaylibVec2) -> Self {
        Self {
            x: value.x as f64,
            y: value.y as f64,
        }
    }
}

impl From<Vector2> for RaylibVec2 {
    fn from(value: Vector2) -> Self {
        RaylibVec2 {
            x: value.x as f32,
            y: value.y as f32,
        }
    }
}

impl From<[f64; 2]> for Vector2 {
    fn from(value: [f64; 2]) -> Self {
        Vector2 {
            x: value[0],
            y: value[1],
        }
    }
}

impl Vector2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn distance_to(&self, other: Self) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

impl PartialEq for Vector2 {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Add for Vector2 {
    type Output = Vector2;

    fn add(self, rhs: Self) -> Self::Output {
        Vector2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Mul<f64> for Vector2 {
    type Output = Vector2;

    fn mul(self, rhs: f64) -> Self::Output {
        Vector2 {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Mul<Vector2> for f64 {
    type Output = Vector2;

    fn mul(self, rhs: Vector2) -> Self::Output {
        Vector2 {
            x: rhs.x * self,
            y: rhs.y * self,
        }
    }
}

impl Sub for Vector2 {
    type Output = Vector2;

    fn sub(self, rhs: Self) -> Self::Output {
        self + (rhs * -1.)
    }
}
