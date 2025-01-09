package vector3d

import (
	"fmt"
	"math"
)

const Epsilon = 1e-5

type Vec3D struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Quaternion struct {
	S float64
	V Vec3D
}

// Vec3D Functions

// NewVec3D creates a new 3D vector with the given x, y, z components.
func NewVec3D(x, y, z float64) Vec3D {
	return Vec3D{X: x, Y: y, Z: z}
}

// Length calculates the magnitude of the vector.
func (v Vec3D) Length() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

// Normalize scales the vector to have a length of 1, if possible.
func (v Vec3D) Normalize() Vec3D {
	length := v.Length()
	if length >= Epsilon {
		return v.Div(length)
	}
	return v
}

// Cross calculates the cross product of two vectors.
func (v Vec3D) Cross(other Vec3D) Vec3D {
	return Vec3D{
		X: v.Y*other.Z - v.Z*other.Y,
		Y: v.Z*other.X - v.X*other.Z,
		Z: v.X*other.Y - v.Y*other.X,
	}
}

// MultiplyScalar scales the vector by a scalar value.
func (v Vec3D) MultiplyScalar(scalar float64) Vec3D {
	return Vec3D{
		X: v.X * scalar,
		Y: v.Y * scalar,
		Z: v.Z * scalar,
	}
}

// ScalarProduct calculates the dot product of two vectors.
func (v Vec3D) ScalarProduct(other Vec3D) float64 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

// Subtract subtracts another vector from this vector.
func (v Vec3D) Subtract(other Vec3D) Vec3D {
	return Vec3D{
		X: v.X - other.X,
		Y: v.Y - other.Y,
		Z: v.Z - other.Z,
	}
}

// DivideScalar divides the vector by a scalar value.
func (v Vec3D) DivideScalar(scalar float64) Vec3D {
	return v.MultiplyScalar(1.0 / scalar)
}

// Add adds another vector to this vector.
func (v Vec3D) Add(other Vec3D) Vec3D {
	return Vec3D{
		X: v.X + other.X,
		Y: v.Y + other.Y,
		Z: v.Z + other.Z,
	}
}

// Div divides the vector by a scalar value and returns the resulting vector.
func (v Vec3D) Div(scalar float64) Vec3D {
	return v.DivideScalar(scalar)
}

// Angle calculates the angle (in degrees) between this vector and another vector.
func (v Vec3D) Angle(other Vec3D) float64 {
	dotProduct := v.ScalarProduct(other)
	lengths := v.Length() * other.Length()
	if lengths < Epsilon {
		return 0
	}
	return RadToDeg(math.Acos(dotProduct / lengths))
}

// RadToDeg converts radians to degrees.
func RadToDeg(radians float64) float64 {
	return radians * 180.0 / math.Pi
}

// DegToRad converts degrees to radians.
func DegToRad(degrees float64) float64 {
	return degrees * math.Pi / 180.0
}

// Print outputs the vector in a formatted string.
func (v Vec3D) Print() {
	fmt.Printf("[%.1f/%.1f/%.1f]\n", v.X, v.Y, v.Z)
}

// Quaternion Functions

// NewQuaternion creates a quaternion from an axis and an angle.
func NewQuaternion(axis Vec3D, angle float64) Quaternion {
	return Quaternion{
		S: math.Cos(angle / 2.0),
		V: axis.MultiplyScalar(math.Sin(angle / 2.0)),
	}
}

// Multiply performs quaternion multiplication (non-commutative).
func (q Quaternion) Multiply(other Quaternion) Quaternion {
	return Quaternion{
		S: q.S*other.S - q.V.ScalarProduct(other.V),
		V: q.V.Cross(other.V).
			Add(other.V.MultiplyScalar(q.S)).
			Add(q.V.MultiplyScalar(other.S)),
	}
}

// MultiplyScalar scales the quaternion by a scalar value.
func (q Quaternion) MultiplyScalar(scalar float64) Quaternion {
	return Quaternion{
		S: q.S * scalar,
		V: q.V.MultiplyScalar(scalar),
	}
}

// Add adds two quaternions together.
func (q Quaternion) Add(other Quaternion) Quaternion {
	return Quaternion{
		S: q.S + other.S,
		V: q.V.Add(other.V),
	}
}

// Subtract subtracts another quaternion from this quaternion.
func (q Quaternion) Subtract(other Quaternion) Quaternion {
	return Quaternion{
		S: q.S - other.S,
		V: q.V.Subtract(other.V),
	}
}

// Conjugate returns the conjugate of the quaternion.
func (q Quaternion) Conjugate() Quaternion {
	return Quaternion{
		S: q.S,
		V: q.V.MultiplyScalar(-1.0),
	}
}

// Inverse calculates the inverse of the quaternion.
func (q Quaternion) Inverse() Quaternion {
	lengthSquared := q.Length() * q.Length()
	if lengthSquared < Epsilon {
		return q // Avoid division by zero
	}
	return q.Conjugate().MultiplyScalar(1.0 / lengthSquared)
}

// Normalize scales the quaternion to have a length of 1, if possible.
func (q Quaternion) Normalize() Quaternion {
	length := q.Length()
	if length < Epsilon {
		return q
	}
	return Quaternion{
		S: q.S / length,
		V: q.V.MultiplyScalar(1.0 / length),
	}
}

// Length calculates the magnitude of the quaternion.
func (q Quaternion) Length() float64 {
	return math.Sqrt(q.S*q.S + q.V.X*q.V.X + q.V.Y*q.V.Y + q.V.Z*q.V.Z)
}

// IsNormalized checks if the quaternion is normalized.
func (q Quaternion) IsNormalized() bool {
	lengthSquared := q.S*q.S + q.V.X*q.V.X + q.V.Y*q.V.Y + q.V.Z*q.V.Z
	return math.Abs(lengthSquared-1.0) <= Epsilon
}

// RotatePointWithQuaternion rotates a point using the quaternion.
func RotatePointWithQuaternion(q Quaternion, point Vec3D) Vec3D {
	normQ := q.Normalize()
	pointQ := Quaternion{S: 0.0, V: point}

	rotatedQ := normQ.Multiply(pointQ).Multiply(normQ.Inverse())
	return rotatedQ.V
}

// RotatePointAroundAxis rotates a point around an axis by a given angle.
func RotatePointAroundAxis(axis Vec3D, angle float64, point Vec3D) Vec3D {
	q := NewQuaternion(axis, angle)
	return RotatePointWithQuaternion(q, point)
}
