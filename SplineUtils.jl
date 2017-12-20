module SplineUtils

using GeomUtils

export
    B_Spline,
    B_SplineDerivative,
    ResampleSplineByArcLength,
    Bspline,
    ResampleByArcLength!,
    ComputeFrenetCoordsFromCartesian,
    Dubbins,
    SplineFromEndPoints,
    ExtendedDubbins,
    TransitionSpline

"""
Computes the coefficients of a B-spline of degree "degree" from the control points "Pts", and
samples "L_tt" points from that spline
"""
function B_Spline(Pts, degree, L_tt) # returns spline and derivative of spline
    k = degree
    p = k - 1 # degree of derivative
    n = size(Pts,2) # number of control points
    m = n + k + 1; # number of knots in T

    #knots
    T = 0.5*ones(m,1)
    T[1:k] = 0
    T[(k+1):end-(k)] = linspace(0,1,m-2*k)
    T[end-(k):end] = 1.0

    tt = linspace(0,1,L_tt)
    rx = zeros(size(tt))
    ry = zeros(size(tt))

    invd = Dict() # inverse denominator (to deal with zero denominators)
    for i = 1:k+1
        invd[i] = T[(i+1):end] - T[1:(end-(i))]
        # take care of empty knot spans
        for j = 1:size(invd[i])[1]
            if invd[i][j] != 0
                invd[i][j] = 1.0/invd[i][j]
            end
        end
    end

    N = Dict()
    for j = 1:size(tt)[1]
        t = tt[j]
        N[1] = 1.0*(t .>= T[1:end-1]) .* (t .< T[2:end])
        for i = 1:k+1
            if i > 1
                N[i] = (t - T[1:end-i]) .* (invd[i-1].*N[i-1])[1:end-1] + (T[i+1:end] - t) .*(invd[i-1].*N[i-1])[2:end]
            end
        end
        rx[j] = (Pts[1,:]'*N[k+1])[1]
        ry[j] = (Pts[2,:]'*N[k+1])[1]
    end

    rx[end] = Pts[1,end]
    ry[end] = Pts[2,end]

    return T, tt, rx, ry
end

#####################################################
"""
Computes the derivative of a spline
"""
function B_SplineDerivative(T, # knots
    tt, # sample locations t ∈ [0,1.0]
    Pts, # control points
    k) # degree of spline

    p = k - 1 # degree of derivative
    n = size(Pts,2) # number of control points
    m = n + k + 1; # number of knots in T

    invd = Dict() # inverse denominator (to deal with zero denominators)
    for i = 1:k+1
        invd[i] = T[(i+1):end] - T[1:(end-(i))]
        # take care of empty knot spans
        for j = 1:size(invd[i])[1]
            if invd[i][j] != 0
                invd[i][j] = 1.0/invd[i][j]
            end
        end
    end

    Q = zeros(size(Pts[:,1:end-1]))
    ṙx = zeros(size(tt)) # derivatives
    ṙy = zeros(size(tt)) # derivatives

    # Derivative
    Q[1,:] = p*invd[1][p+1:end-p].*(Pts[1,2:end] - Pts[1,1:end-1])
    Q[2,:] = p*invd[1][p+1:end-p].*(Pts[2,2:end] - Pts[2,1:end-1])

    N = Dict()
    for j = 1:size(tt)[1]
        t = tt[j]
        N[1] = 1.0*(t .>= T[1:end-1]) .* (t .< T[2:end])
        for i = 1:k+1
            if i > 1
                N[i] = (t - T[1:end-i]) .* (invd[i-1].*N[i-1])[1:end-1] + (T[i+1:end] - t) .*(invd[i-1].*N[i-1])[2:end]
            end
        end

        ṙx[j] = (Q[1,:]'*N[k][2:end-1])[1] # derivatives
        ṙy[j] = (Q[2,:]'*N[k][2:end-1])[1] # derivatives
    end

    ṙx[1] = 0.0001 * sign(ṙx[2])
    ṙx[end] = 0.0001 * sign(ṙx[end-1])
    ṙy[1] = 0.0001 * sign(ṙy[2])
    ṙy[end] = 0.0001 * sign(ṙy[end-1])

    return ṙx, ṙy
end

#################################################
"""
Evenly resamples spline at num_samples points along arc_length
"""
function ResampleSplineByArcLength(s,rx,ry,θ,k,num_samples; sP=nothing)
    if sP==nothing
    	sP = collect(linspace(0,s[end],num_samples))
    end
    xP = zeros(num_samples)
    yP = zeros(num_samples)
    θP = zeros(num_samples)
    kP = zeros(num_samples)
    # ttP = zeros(num_samples)

    xP[1] = rx[1]
    xP[end] = rx[end]
    yP[1] = ry[1]
    yP[end] = ry[end]
    θP[1] = θ[1]
    θP[end] = θ[end]
    kP[1] = k[1]
    kP[end] = k[end]
    # ttP[end] = 1.0

    j = 1
    for j in 1:num_samples
        for t in 2:length(s)
            if  s[t-1] <= sP[j] && sP[j] < s[t]
                scale = (sP[j] - s[t-1]) / (s[t] - s[t-1])
                xP[j] = rx[t-1] + scale * (rx[t] - rx[t-1])
                yP[j] = ry[t-1] + scale * (ry[t] - ry[t-1])
                # θP[j] = θ[t-1] + scale * ((θ[t] - θ[t-1]))
                θP[j] = θ[t-1] + scale * ((θ[t] - θ[t-1])*(abs(θ[t]-θ[t-1]) < π) + (-2π+abs(θ[t]-θ[t-1]))*(abs(θ[t]-θ[t-1]) > π))
                kP[j] = k[t-1] + scale * (k[t] - k[t-1])
                # ttP[j] = (t - 1 + scale) / length(s)
                break
            end
        end
    end

    return sP, xP, yP, θP, kP
end

function TransitionSpline(npts;y₁=0,y₂=0,L=1,B=0.5)
    """
    generates a spline that smoothly transitions between y₁ and y₂
    """
    B = min(B, 0.5)
    ctrl_pts = [
        0.0 B*L (1-B)*L 1*L;
        y₁ y₁ y₂ y₂
    ]
    spline = Bspline(ctrl_pts, 3, npts;resample=false)
    _,_,y,_,_ = ResampleSplineByArcLength(spline.s, spline.x, spline.y,
    spline.θ, spline.k, npts)
    return y
end

"""
Bspline type
"""
type Bspline
    knots
    controlPts
    degree
    numPts
    x
    y
    θ
    s
    k
    function Bspline(controlPts, degree, numPts; resample=true, Δs=1.0)
        spline = new()
        spline.controlPts = controlPts
        spline.degree = degree
        spline.numPts = numPts

        T, tt, x, y = B_Spline(controlPts, degree, numPts)
        Δx = diff(x)
        Δy = diff(y)
        s = vcat([0], cumsum(sqrt.(Δx.^2 + Δy.^2)))
        prepend!(Δx,Δx[1])
        append!(Δx,Δx[end])
        prepend!(Δy,Δy[1])
        append!(Δy,Δy[end])
        Δx = (Δx[2:end] .+ Δx[1:end-1]) / 2
        Δy = (Δy[2:end] .+ Δy[1:end-1]) / 2
        θ = atan2.(Δy,Δx)
        θ[1] = atan2(controlPts[2,2]-controlPts[2,1],controlPts[1,2]-controlPts[1,1])
        θ[end] = atan2(controlPts[2,end]-controlPts[2,end-1],controlPts[1,end]-controlPts[1,end-1])
        k = diff(θ)./diff(s) # curvature
        prepend!(k, k[1])
        append!(k, k[end])
        k = (k[2:end] .+ k[1:end-1]) / 2

        spline.knots = T
        spline.x = x
        spline.y = y
        spline.θ = θ
        spline.s = s
        spline.k = k

        if resample == true
            ResampleByArcLength!(spline; Δs=Δs)
        end

        return spline
    end
end

function ComputeFrenetCoordsFromCartesian(x,y;θ₁=nothing,θ₂=nothing)
    Δx = diff(x)
    Δy = diff(y)
    s = vcat([0], cumsum(sqrt.(Δx.^2 + Δy.^2)))
    prepend!(Δx,Δx[1])
    append!(Δx,Δx[end])
    prepend!(Δy,Δy[1])
    append!(Δy,Δy[end])
    Δx = (Δx[2:end] .+ Δx[1:end-1]) / 2
    Δy = (Δy[2:end] .+ Δy[1:end-1]) / 2
    θ = atan2.(Δy,Δx)
    if θ₁ != nothing
        θ[1] = θ₁
    end
    if θ₂ != nothing
        θ[end] = θ₂
    end
    k = diff(θ)./diff(s) # curvature
    prepend!(k, k[1])
    append!(k, k[end])
    k = (k[2:end] .+ k[1:end-1]) / 2

    return s, θ, k
end

"""
ResampleByArcLength!()
resamples points of spline at approximate fixed arc length intervals defined by
Δs
"""
function ResampleByArcLength!(spline; Δs=0.5)
    num_samples = Int(div(spline.s[end], Δs))
    spline.s,spline.x,spline.y,spline.θ,spline.k = ResampleSplineByArcLength(
        spline.s,spline.x,spline.y,spline.θ,spline.k, Int(div(spline.s[end],Δs)))
end

function Dubbins(pt₁,pt₂,θ₁,θ₂; Δs = 0.5)
    """
    returns points along Dubbins curve from Pt1 to Pt2, sampled at intervals of Δŝ ≋ Δs
    """
    Δθ = θ₂ - θ₁
    vecA = [cos(θ₁);sin(θ₁)]*(pt₂-pt₁)'*[cos(θ₁);sin(θ₁)]
    vecB = (pt₂-pt₁) - vecA
    vecD = [cos(θ₂);sin(θ₂)]*norm(vecB)/abs(sin(Δθ))
    vecC = (pt₂-pt₁) - vecD

    p₁ = pt₁
    p₄ = pt₂
    if norm(vecC) < norm(vecD)
        p₂ = pt₁
        p₃ = pt₂ - vecD*(1-norm(vecC)/norm(vecD))
    elseif norm(vecD) < norm(vecC)
        p₂ = pt₁ + vecC * (1-norm(vecD)/norm(vecC))
        p₃ = pt₂
    else
        p₂ = pt₁
        p₃ = pt₂
    end

    A = (p₃-p₂)'*[cos(θ₁);sin(θ₁)]
    C = A / sin(θ₂-θ₁)
    X = Rot(θ₁)*[0;C] + p₂  # <==== Center point
    Δ₁ = p₂ - X
    Δ₂ = p₃ - X
    r = norm(Δ₁)
    ϕ₁ = atan2(Δ₁[2],Δ₁[1])
    ϕ₂ = atan2(Δ₂[2],Δ₂[1])
    Δϕ = mod(ϕ₂,2*π)-mod(ϕ₁,2*π)
    if abs(Δϕ) > π
        Δϕ -= sign(Δϕ)*2*π
    end
    vec1 = X + norm(Δ₁)*[cos(ϕ₁);sin(ϕ₁)]
    vec2 = X + norm(Δ₂)*[cos(ϕ₂);sin(ϕ₂)]

    L = norm(p₂-p₁) + abs(r*Δϕ) + norm(p₄-p₃)
    Δŝ = L / ceil(L/Δs)
    # straight line p₁ -> p₂
    pts = p₁
    L₁₂ = norm(p₂-p₁)
    ds = 0
    n = Int(div(L₁₂, Δŝ))
    if n > 0
        pts = pts[:,end] .+ ((p₂-p₁)/L₁₂) * collect(linspace(ds, ds+n*Δŝ, n+1))'
        ds = Δŝ - mod(L₁₂, Δŝ)
    end
    # arc p₁ -> p₂
    ϕ₁ += sign(Δϕ)*ds/r
    Δϕ -= sign(Δϕ)*ds/r
    n = Int(div(abs(Δϕ*r), Δŝ))
    if n > 0
        ϕ = collect(linspace(ϕ₁,ϕ₁+n*sign(Δϕ)*Δŝ/r,n+1))
        pts = hcat(pts, X .+ r * hcat(cos.(ϕ), sin.(ϕ))')
        dϕ = mod(ϕ[end] - ϕ₂,2*π)
        ds = -sign(dϕ) * min(abs(dϕ), abs(2π-dϕ)) * r + Δŝ
    end
    # straight line p₃ -> p₄
    L₃₄ = norm(p₄-p₃)
    n = Int(div(L₃₄ - ds, Δŝ))
    if n > 0
        pts = hcat(pts, p₃ .+ ((p₄-p₃)/L₃₄) * collect(linspace(ds, ds+n*Δŝ, n+1))')
        ds = -mod(L₃₄ - ds, Δŝ)
    end

    if norm(pts[:,end]-pt₂) > 0.5*Δs
        pts = hcat(pts,pt₂)
    end

    x = pts[1,:]
    y = pts[2,:]
    return x,y
end

function ExtendedDubbins(pt₀,pt₁,pt₂,pt₃; θ₁=nothing, θ₂=nothing, Δs = 0.5)
    """
    returns evenly sampled curve from from pt₀ -> pt₁ -> pt₂ -> pt₃,
    sampled at intervals of Δŝ ≋ Δs. The section from pt₁ -> pt₂ is a
    Dubbins curve as defined in the Dubbins() function
    """
    if θ₁ == nothing
        θ₁ = atan2((pt₁-pt₀)[2],(pt₁-pt₀)[1])
    end
    if θ₂ == nothing
        θ₂ = atan2((pt₃-pt₂)[2],(pt₃-pt₂)[1])
    end
    Δθ = θ₂ - θ₁
    vecA = [cos(θ₁);sin(θ₁)]*(pt₂-pt₁)'*[cos(θ₁);sin(θ₁)]
    vecB = (pt₂-pt₁) - vecA
    vecD = [cos(θ₂);sin(θ₂)]*norm(vecB)/abs(sin(Δθ))
    vecC = (pt₂-pt₁) - vecD

    # p₀ = pt₀
    p₁ = pt₀
    p₄ = pt₃
    # p₅ = pt₃
    if norm(vecC) < norm(vecD)
        p₂ = pt₁
        p₃ = pt₂ - vecD*(1-norm(vecC)/norm(vecD))
    elseif norm(vecD) < norm(vecC)
        p₂ = pt₁ + vecC * (1-norm(vecD)/norm(vecC))
        p₃ = pt₂
    else
        p₂ = pt₁
        p₃ = pt₂
    end

    A = (p₃-p₂)'*[cos(θ₁);sin(θ₁)]
    C = A / sin(θ₂-θ₁)
    X = Rot(θ₁)*[0;C] + p₂  # <==== Center point
    Δ₁ = p₂ - X
    Δ₂ = p₃ - X
    r = norm(Δ₁)
    ϕ₁ = atan2(Δ₁[2],Δ₁[1])
    ϕ₂ = atan2(Δ₂[2],Δ₂[1])
    Δϕ = mod(ϕ₂,2*π)-mod(ϕ₁,2*π)
    if abs(Δϕ) > π
        Δϕ -= sign(Δϕ)*2*π
    end
    vec1 = X + norm(Δ₁)*[cos(ϕ₁);sin(ϕ₁)]
    vec2 = X + norm(Δ₂)*[cos(ϕ₂);sin(ϕ₂)]

    L = norm(p₂-p₁) + abs(r*Δϕ) + norm(p₄-p₃)
    Δŝ = L / ceil(L/Δs)
    # straight line p₁ -> p₂
    pts = p₁
    ds = 0
    L₁₂ = norm(p₂-p₁)
    n = Int(div(L₁₂, Δŝ))
    if n > 0
        pts = pts[:,end] .+ ((p₂-p₁)/L₁₂) * collect(linspace(ds, ds+n*Δŝ, n+1))'
        ds = Δŝ - mod(L₁₂, Δŝ)
    end
    # arc p₁ -> p₂
    ϕ₁ += sign(Δϕ)*ds/r
    Δϕ -= sign(Δϕ)*ds/r
    n = Int(div(abs(Δϕ*r), Δŝ))
    if n > 0
        ϕ = collect(linspace(ϕ₁,ϕ₁+n*sign(Δϕ)*Δŝ/r,n+1))
        pts = hcat(pts, X .+ r * hcat(cos.(ϕ), sin.(ϕ))')
        dϕ = mod(ϕ[end] - ϕ₂,2*π)
        ds = -sign(dϕ) * min(abs(dϕ), abs(2π-dϕ)) * r + Δŝ
    end
    # straight line p₃ -> p₄
    L₃₄ = norm(p₄-p₃)
    n = Int(div(L₃₄ - ds, Δŝ))
    if n > 0
        pts = hcat(pts, p₃ .+ ((p₄-p₃)/L₃₄) * collect(linspace(ds, ds+n*Δŝ, n+1))')
        ds = -mod(L₃₄ - ds, Δŝ)
    end

    if norm(pts[:,end]-pt₃) > 0.5*Δs
        pts = hcat(pts,pt₃)
    end

    x = pts[1,:]
    y = pts[2,:]
    return x,y
end

function SplineFromEndPoints(pt₁, pt₂, θ₁, θ₂; numPts=100, degree=3, resample=true, Δs=0.5)
    """
    generates a spline that leaves pt1 at heading θ₁, and arrives
    at pt2 with heading θ₂
    """
    d = norm(pt₁-pt₂)
    V₁ = (d/4) * Rot(θ₁) * [
        0.0 1.0;
        0.0 0.0
    ] .+ pt₁
    V₂ = (d/4) * Rot(θ₂+π) * [
        1.0 0.0;
        0.0 0.0
    ] .+ pt₂
    Pts = hcat(V₁,(V₁[:,end]+V₂[:,1])/2,V₂)
    spline = Bspline(Pts, degree, numPts; resample=resample, Δs=Δs)
    return spline
end

end # module
