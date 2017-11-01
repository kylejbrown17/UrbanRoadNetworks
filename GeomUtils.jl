module GeomUtils

export
    Rot,
    unitVec

Rot(θ) = [cos(θ) -sin(θ); sin(θ) cos(θ)]
unitVec(θ) = [cos(θ);sin(θ)]
end # module
