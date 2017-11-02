module RoadwayNetworks

using SplineUtils
using GeomUtils
using LightGraphs

export
    Curve,
    curvePt,
    Lane,
    Road,
    Intersection,
    UrbanMap,
    id_dispenser,
    getId!,
    Left,
    Right,
    Opposite,
    populateIntersection!,
    ConstructRoadFromReference!,
    GenerateRoadBetweenIntersections!,
    GenerateDoubleRoadBetweenIntersections!


Left = Dict(:North => :West, :East => :North, :South => :East, :West => :South)
Right = Dict(:North => :East, :East => :South, :South => :West, :West => :North)
Opposite = Dict(:North => :South, :East => :West, :South => :North, :West => :East)

type Curve
    x
    y
    s
    θ
    k
    function Curve(x,y;s=nothing,θ=nothing,k=nothing)
        curve = new()
        curve.x = x
        curve.y = y
        if s == nothing || θ == nothing || k == nothing
            curve.s, curve.θ, curve.k = ComputeFrenetCoordsFromCartesian(x,y)
        else
            curve.s, curve.θ, curve.k = s, θ, k
        end
        return curve
    end
end

type curvePt
    id
    parent_id
    x
    y
    θ # heading
    s # arc length
    k # curvature
    function curvePt(;id=nothing,parent_id=nothingx=nothing,y=nothing,s=nothing,θ=nothing,k=nothing)
        pt = new()
        pt.id = id
        parent_id = parent_id
        pt.x = x
        pt.y = y
        pt.s = s
        pt.θ = θ
        pt.k = k
        return pt
    end
end

type Road
    id
    referenceLine
    lanes
    successors # other roads
    predecessors # other roads
    speed
    function Road(id;curve=nothing,speed=nothing)
        road = new()
        road.id = id
        road.referenceLine = curve
        road.lanes = Set()
        road.successors = nothing
        road.predecessors = nothing
        road.speed = speed
        return road
    end
end

type Lane
    id
    centerLine # geometry (x,y,s,θ,k)
    length
    width
    road
    boundaryLeft # boundaryID
    boundaryRight # boundaryID
    predecessors # laneID
    successors # laneID
    neighborsLeft # laneID
    neighborsRight # laneID
    signal_state # :Green, :Yellow or :Red
    function Lane(lane_id,x,y,s,θ,k;width=nothing,road_id=nothing)
        lane = new()
        lane.id = lane_id
        lane.road = road_id
        lane.centerLine = Curve(x,y;s=s,θ=θ,k=k)
        lane.length = lane.centerLine.s[end]
        lane.width = width
        lane.boundaryLeft = nothing
        lane.boundaryRight = nothing
        lane.predecessors = Set()
        lane.successors = Set()
        lane.neighborsLeft = Set()
        lane.neighborsRight = Set()
        lane.signal_state = :Green
        return lane
    end
    function Lane(lane_id,curve::Curve;width=nothing,road_id=nothing)
        lane = new()
        lane.id = lane_id
        lane.road = road_id
        lane.centerLine = curve
        lane.length = lane.centerLine.s[end]
        lane.width = width
        lane.boundaryLeft = nothing
        lane.boundaryRight = nothing
        lane.predecessors = Set()
        lane.successors = Set()
        lane.neighborsLeft = Set()
        lane.neighborsRight = Set()
        lane.signal_state = :Green
        return lane
    end
end

# type TrafficSignal
#     states # ordered list of traffic states through which the signal passes
#     current_state
#     timing # list of times corresponding to each state change
#     t # time since last change
#     function TrafficSignal(intersection::Intersection)
#         signal = new()
#         signal.states = []
#         # for dir in [:North, :East, :South, :West]
#         #     for i,k in enumerate(sort([k for (k, v) in intersection.entrances[dir]]))
#         #
#         #     end
#         # end
#         timing = []
#         t = 0
#         return signal
#     end
# end

type Intersection
    id
    x
    y
    θ
    LanesIn
    LanesOut
    # Road level connections
    connections_out # Dict(:North, :East, :South, :West) => road_id
    connections_in # Dict(:North, :East, :South, :West) => road_id
    # Lane level connections
    entrances # Dict(:North, :East, :South, :West) => Dict(-1, 0, 1, ..., offset) => [ID(s)] of associated lanes
    exits # Dict(:North, :East, :South, :West) => Dict(-1, 0, 1, ..., offset) => [ID(s)] of associated lanes
    refpoints # Dict(:North, :East, :South, :West) => Dict(:in,:out) => [x;y]
    entrance_pts
    exit_pts
    lanes # Dict(id) => Lane
    roads
    function Intersection(id,x,y,θ₁,θ₂;LanesIn=nothing)
        retval = new()
        retval.id = id
        retval.x = x
        retval.y = y
        retval.θ = Dict(:East => θ₁, :North => θ₂, :West => θ₁ + π, :South => θ₂ + π)
        retval.connections_out = Dict( k => -1 for k in [:North, :East, :South, :West])
        retval.connections_in = Dict( k => -1 for k in [:North, :East, :South, :West])
        retval.LanesIn = LanesIn
        retval.LanesOut = nothing
        retval.entrances = Dict(:North => Dict(), :South => Dict(), :East => Dict(), :West => Dict())
        retval.exits = Dict(:North => Dict(), :South => Dict(), :East => Dict(), :West => Dict())
        retval.entrance_pts = Dict(:North => Dict(), :South => Dict(), :East => Dict(), :West => Dict())
        retval.exit_pts = Dict(:North => Dict(), :South => Dict(), :East => Dict(), :West => Dict())
        retval.refpoints = Dict(:North => Dict(), :South => Dict(), :East => Dict(), :West => Dict())
        retval.lanes = Dict(:North => Dict(), :South => Dict(), :East => Dict(), :West => Dict())
        retval.roads = Dict(:North => Dict(), :South => Dict(), :East => Dict(), :West => Dict())
        return retval
    end
end

type id_dispenser
    """
    id_dispenser
    holds a set of available ids and dispenses them upon request via getId!()
    """
    available_ids
    function id_dispenser(;num_ids=1000000)
        retval = new()
        retval.available_ids = Set(i for i in 1:num_ids)
        return retval
    end
end

function getId!(dispenser::id_dispenser)
    """
    returns a single id from an id_dispenser object
    """
    return pop!(dispenser.available_ids)
end

type UrbanMap
    roads
    lanes
    intersections
    points
    dispenser
    lane_graph
    distance_matrix # sparse distance matrix
    function UrbanMap(;numElements=1000000)
        urbanMap = new()
        urbanMap.roads = Dict()
        urbanMap.lanes = Dict()
        urbanMap.intersections = Dict()
        urbanMap.points = Dict()
        urbanMap.dispenser = id_dispenser()
        lane_graph = DiGraph()
        return urbanMap
    end
end

function populateDistanceMatrix!(urbanMap::UrbanMap)
    # populate all distances between lanes in urbanMap.distance_matrix
end

function getId!(urbanMap::UrbanMap)
    """
    returns a single id from an id_dispenser object
    """
    return pop!(urbanMap.dispenser.available_ids)
end

function populateIntersection!(intersection,urbanMap;LanesIn=nothing,width=1,buffer=1,Δs=0.5)
    """
    Populates the geometry and metadata of an intersection from a list of incoming lanes and transitions
    """
    w = width
    b = buffer

    x₀ = intersection.x
    y₀ = intersection.y

    if LanesIn == nothing
        LanesIn = intersection.LanesIn
    else
        intersection.LanesIn = LanesIn
    end

    Directions = [:North, :East, :South, :West]
#     for k in Directions
#         road = Road(getId!(urbanMap))
#     end

    θ = intersection.θ
    # account for shearing when Δθ is not exactly 90 degrees
    τ = abs(1.0 / dot([cos(θ[:East]);sin(θ[:East])],[cos(θ[:North]-π/2);sin(θ[:North]-π/2)]))
    w = w * τ
    LanesOut = Dict( k => [Set([:Straight]) for i in 1:max(
                    sum([Int(:Straight in lane) for lane in LanesIn[k]]),
                    sum([Int(:Left in lane) for lane in LanesIn[Right[k]]]),
                    sum([Int(:Right in lane) for lane in LanesIn[Left[k]]]),)] for k in Directions)
    D = Dict( k => max(length(LanesIn[k]) + length(LanesOut[Opposite[k]]),
            length(LanesIn[Opposite[k]]) + length(LanesOut[k])) for k in Directions)
    # incoming lanes - lateral position begins from the right side
    PositionsIn = Dict( k => [i for i in 1:length(LanesIn[k])] for k in Directions)
    # reference positions - for specifying incoming roadway geometry
    PositionRefs = Dict(
        k => sum([Int(:Straight in lane || :Right in lane) for lane in LanesIn[k]]) for k in Directions)
    # outgoing lanes - lateral position counts down from left side (leftover positions are empty--no lanes in them)
    PositionsOut = Dict( k => [D[k] + 1 - i for i in 1:length(LanesOut[k])] for k in Directions)
    d = Dict( k => w*D[k]/2 + b for k in Directions)
    d_vecs = Dict( k => d[Right[k]] * Rot(θ[k]) * [1;0] for k in Directions)
    ctrs = Dict( k => d_vecs[k] + [x₀;y₀] for k in Directions)
    corners = Dict((k, Left[k]) => d_vecs[k] + d_vecs[Left[k]] + [x₀;y₀] for k in Directions)
    # entry points
    CurveStartPoints = Dict( k => [(corners[Opposite[k], Right[k]] .+ normalize(
                    d_vecs[Left[k]])*(w*(p-.5) + b)) for p in PositionsIn[k]] for k in Directions)
    # exit points
    CurveEndPoints = Dict(k => [(corners[(k, Left[k])] .+ normalize(
                    d_vecs[Right[k]])*(w*(p.-.5) + b)) for p in PositionsOut[k]] for k in Directions)

    # perpendicular entrance and exit points for appropriate connection geometry
    EntrancePoints = Dict()
    ExitPoints = Dict()
    RefPts_In = Dict()
    RefPts_Out = Dict()
    for k in Directions
        v₁ = [cos(θ[k]);sin(θ[k])]
        v₂ = [cos(θ[Right[k]]);sin(θ[Right[k]])]
        if dot(v₁, v₂) > 0
            θₚ = θ[k] - π/2
            vₚ = [cos(θₚ);sin(θₚ)]
            v₀ = corners[(Left[k], Opposite[k])]
            EntrancePoints[k] = [ v₀ + vₚ*dot(vₚ, v-v₀) for v in CurveStartPoints[k]]
            ExitPoints[Opposite[k]] = [ v₀+vₚ *  dot(vₚ, v-v₀) for v in CurveEndPoints[Opposite[k]]]
            RefPts_In[k] = EntrancePoints[k][PositionRefs[k]] .- vₚ*w/(2τ)
            RefPts_Out[Opposite[k]] = ExitPoints[Opposite[k]][end] .+ vₚ*w/(2τ)
        else
            θₚ = θ[k] + π/2
            vₚ = [cos(θₚ);sin(θₚ)]
            v₀ = corners[(Opposite[k], Right[k])]
            EntrancePoints[k] = [ v₀ + vₚ *  dot(vₚ, v-v₀) for v in CurveStartPoints[k]]
            ExitPoints[Opposite[k]] = [ v₀ + vₚ *  dot(vₚ, v-v₀) for v in CurveEndPoints[Opposite[k]]]
            RefPts_In[k] = EntrancePoints[k][PositionRefs[k]] .+ vₚ*w/(2τ)
            RefPts_Out[Opposite[k]] = ExitPoints[Opposite[k]][end] .- vₚ*w/(2τ)
        end
    end
    # intersection ref points
    for (k, pt) in RefPts_In
        intersection.refpoints[k][:in] = pt
    end
    for (k, pt) in RefPts_Out
        intersection.refpoints[k][:out] = pt
    end

    for (j,direction) in enumerate(Directions)
        for (i,lane) in enumerate(reverse(LanesIn[direction]))
            if :Straight in lane
                exit_direction = direction
                pt₀ = EntrancePoints[direction][i]
                pt₁ = CurveStartPoints[direction][i]
                pt₂ = CurveEndPoints[direction][i]
                pt₃ = ExitPoints[direction][i]
#                 x = collect(linspace(pt₁[1],pt₂[1]))
#                 y = collect(linspace(pt₁[2],pt₂[2]))
                x = collect(linspace(pt₀[1],pt₃[1]))
                y = collect(linspace(pt₀[2],pt₃[2]))
                sₗ,θₗ,kₗ = ComputeFrenetCoordsFromCartesian(x,y;θ₁=θ[direction],θ₂=θ[direction])
                id = getId!(urbanMap.dispenser)
                urbanMap.lanes[id] = Lane(id,x,y,sₗ,θₗ,kₗ;width=w)

                offset1 = PositionRefs[direction] - i
                if !haskey(intersection.entrances[direction], offset1)
                    intersection.entrances[direction][offset1] = Set()
                end
                offset2 = PositionRefs[exit_direction] - i
                if !haskey(intersection.exits[exit_direction], offset2)
                    intersection.exits[exit_direction][offset2] = Set()
                end
                push!(intersection.entrances[direction][offset1], id)
                push!(intersection.exits[exit_direction][offset2], id)
                intersection.entrance_pts[direction][offset1] = pt₀
                intersection.exit_pts[exit_direction][offset2] = pt₃
            end
            if :Right in lane
                exit_direction = Right[direction]
                pt₀ = EntrancePoints[direction][i]
                pt₁ = CurveStartPoints[direction][i]
                pt₂ = CurveEndPoints[Right[direction]][i]
                pt₃ = ExitPoints[Right[direction]][i]
#                 x,y = Dubbins(pt₁,pt₂,θ[direction],θ[Right[direction]])
                x,y = ExtendedDubbins(pt₀,pt₁,pt₂,pt₃;θ₁=θ[direction],θ₂=θ[Right[direction]])
                sₗ,θₗ,kₗ = ComputeFrenetCoordsFromCartesian(x,y;θ₁=θ[direction],θ₂=θ[Right[direction]])
                id = getId!(urbanMap.dispenser)
                urbanMap.lanes[id] = Lane(id,x,y,sₗ,θₗ,kₗ;width=w)

                offset1 = PositionRefs[direction] - i
                if !haskey(intersection.entrances[direction], offset1)
                    intersection.entrances[direction][offset1] = Set()
                end
                offset2 = PositionRefs[exit_direction] - i
                if !haskey(intersection.exits[exit_direction], offset2)
                    intersection.exits[exit_direction][offset2] = Set()
                end
                push!(intersection.entrances[direction][offset1], id)
                push!(intersection.exits[exit_direction][offset2], id)
                intersection.entrance_pts[direction][offset1] = pt₀
                intersection.exit_pts[exit_direction][offset2] = pt₃
            end
        end
        for (i,lane) in enumerate(LanesIn[direction])
            if :Left in lane
                exit_direction = Left[direction]
                pt₀ = reverse(EntrancePoints[direction])[i]
                pt₁ = reverse(CurveStartPoints[direction])[i]
                pt₂ = reverse(CurveEndPoints[Left[direction]])[i]
                pt₃ = reverse(ExitPoints[Left[direction]])[i]
#                 x,y = Dubbins(pt₁,pt₂,θ[direction],θ[Left[direction]])
                x,y = ExtendedDubbins(pt₀,pt₁,pt₂,pt₃;θ₁=θ[direction],θ₂=θ[Left[direction]])
                sₗ,θₗ,kₗ = ComputeFrenetCoordsFromCartesian(x,y;θ₁=θ[direction],θ₂=θ[Left[direction]])
                id = getId!(urbanMap.dispenser)
                urbanMap.lanes[id] = Lane(id,x,y,sₗ,θₗ,kₗ;width=w)

                offset1 = PositionRefs[direction] - maximum(PositionsIn[direction]) + i - 1
                if !haskey(intersection.entrances[direction], offset1)
                    intersection.entrances[direction][offset1] = Set()
                end
                offset2 = i - 1
                if !haskey(intersection.exits[exit_direction], offset2)
                    intersection.exits[exit_direction][offset2] = Set()
                end

                push!(intersection.entrances[direction][offset1], id)
                push!(intersection.exits[exit_direction][offset2], id)
                intersection.entrance_pts[direction][offset1] = pt₀
                intersection.exit_pts[exit_direction][offset2] = pt₃
            end
        end
    end
end

function ConstructRoadFromReference!(curve, idxs_in, idxs_out, D_in, D_out, urbanMap, entrances; exits=nothing)
    """
    constructs a lane based on offsets from a reference line

    Arguments:
    curve::Curve:                                      reference line
    idxs_in::Set{Int}                                  Set of incoming lateral offsets from reference
    idxs_out::Set{Int}                                 Set of outgoing lateral offsets from reference
    D_in::Dict{offset::Int, mag::Float}
    D_out::Dict{offset::Int, mag::Float}
    urbanMap::UrbanMap
    entrances::Dict{offset::Int, lane_ids::Set{Int}}
    exits::Dict{offset::Int, lane_ids::Set{Int}}
    """
    road_id = getId!(urbanMap.dispenser)
    road = Road(road_id;curve=curve,speed=50)
    urbanMap.roads[road_id] = road
    XY = hcat(curve.x,curve.y)
    lat_vec = hcat(cos.(curve.θ-π/2),sin.(curve.θ-π/2))
    npts = length(curve.x)

    exit_dict = Dict( idx => Set() for idx in idxs_out)
    neighbor_dict = Dict()

    for idx in union(idxs_in, idxs_out)
        idxA = idx
        idxB = idx
        if !(idxA in idxs_in)
            if idxA > 0
                idxA = maximum(idxs_in)
            else
                idxA = minimum(idxs_in)
            end
        end
        if !(idxB in idxs_out)
            if idxB > 0
                idxB = maximum(idxs_out)
            else
                idxB = minimum(idxs_out)
            end
        end
        transition_spline = TransitionSpline(npts;y₁=D_in[idxA],y₂=D_out[idxB],L=curve.s[end],B=0.5)
        xy = XY .+ transition_spline .* lat_vec
        x = xy[:,1]
        y = xy[:,2]
        s = curve.s
        θ = curve.θ
        _,_,k = ComputeFrenetCoordsFromCartesian(x,y;θ₁=θ[1],θ₂=θ[end])
        # Add lane to graph
        lane_id = getId!(urbanMap.dispenser)
        lane = Lane(lane_id,x,y,s,θ,k; road_id=road_id)
        urbanMap.lanes[lane_id] = lane
        for pred_id in entrances[idxA]
            push!(lane.predecessors, pred_id)
            pred=urbanMap.lanes[pred_id]
            push!(pred.successors, lane_id)
        end
        push!(road.lanes, lane_id)
        push!(exit_dict[idxB], lane_id)
        neighbor_dict[idxA, idxB] = lane_id
    end

    # if exits lanes are supplied, add connections to those too
    if !(exits == nothing)
        for (idx, pred_ids) in exit_dict
            for pred_id in pred_ids
                pred = urbanMap.lanes[pred_id]
                union!(pred.successors, exits[idx])
            end
        end
        for (idx, succ_ids) in exits
            for succ_id in succ_ids
                succ = urbanMap.lanes[succ_id]
                union!(succ.predecessors, exit_dict[idx])
            end
        end
    end

    # Add neighbor connections
    for (idxs, lane_id) in neighbor_dict
        idxA, idxB = idxs
        idx_L = maximum(idxs)-1
        idx_R = minimum(idxs)+1
        lane = urbanMap.lanes[lane_id]
        if haskey(exit_dict, idx_L)
            union!(lane.neighborsLeft, setdiff(exit_dict[idx_L], lane_id))
        end
        if haskey(exit_dict, idx_R)
            union!(lane.neighborsRight, setdiff(exit_dict[idx_R], lane_id))
        end
    end

    return exit_dict
end

function GenerateRoadBetweenIntersections!(urbanMap, I1, I2, d_in, d_out; Δs₁=10, Δs₂=40, speed=50)

    entrances = I1.exits[d_in]
    exits = I2.entrances[d_out]

    idxs_in = Set(k for (k,v) in entrances)
    idxs_out = Set(k for (k,v) in exits)
    idxs_common = intersect(Set(idxs_out), Set(idxs_in))

    ref_pt_in = I1.refpoints[d_in][:out]
    exit_pts_in = I1.exit_pts[d_in]
    D_in = Dict()
    for idx in idxs_in
        D_in[idx] = dot(exit_pts_in[idx] - ref_pt_in, unitVec(I1.θ[d_in]-π/2))
    end

    ref_pt_out = I2.refpoints[d_out][:in]
    entrance_pts_out = I2.entrance_pts[d_out]
    D_out = Dict()
    for idx in idxs_out
        D_out[idx] = dot(entrance_pts_out[idx] - ref_pt_out, unitVec(I2.θ[d_out]-π/2))
    end

    spline = SplineFromEndPoints(ref_pt_in, ref_pt_out, I1.θ[d_in], I2.θ[d_out]);
    ref_line = Curve(spline.x, spline.y; s=spline.s, θ=spline.θ, k=spline.k)
    S = ref_line.s[end]
    ds = mean(diff(ref_line.s))

    if S - 2 * (Δs₁ + Δs₂) < 1
        # construct 3 roads
        # SEGMENT 1 - STRAIGHT SECTION
        p₁ = 1
        p₂ = Int(round(Δs₁ / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_in, D_in, D_in, urbanMap, entrances)
        # SEGMENT 2 - FAN SECTION
        p₁ = p₂
        p₂ = Int(round((S-Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_out, D_in, D_out, urbanMap, connections)
        # SEGMENT 3 - STRAIGHT
        p₁ = p₂
        p₂ = length(ref_line.s)
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_out, idxs_out, D_out, D_out, urbanMap, connections; exits=exits);
    else
        # construct 5 roads: straight, fan in, straight, fan out, straight
        # SEGMENT 1 - STRAIGHT SECTION
        p₁ = 1
        p₂ = Int(round(Δs₁ / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_in, D_in, D_in, urbanMap, entrances)
        # SEGMENT 2 - FAN SECTION
        p₁ = p₂
        p₂ = Int(round((Δs₂+Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_common, D_in, D_in, urbanMap, connections)
        # SEGMENT 3 - STRAIGHT
        Δs₃ = S - 2 * (Δs₁ + Δs₂) # straight section length
        p₁ = p₂
        p₂ = Int(round((Δs₃+Δs₂+Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_common, idxs_common, D_in, D_out, urbanMap, connections)
        # SECTION 4 - FAN OUT
        Δs₄ = Δs₂ # fan out section length
        p₁ = p₂
        p₂ = Int(round((Δs₄+Δs₃+Δs₂+Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_common, idxs_out, D_out, D_out, urbanMap, connections)
        # SECTION 4 - STRAIGHT
        p₁ = p₂
        p₂ = length(ref_line.s)
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_out, idxs_out, D_out, D_out, urbanMap, connections; exits=exits);
    end
end

function GenerateRoadBetweenIntersections!(urbanMap, I1, I2, d_in, d_out, ref_line; Δs₁=10, Δs₂=40, speed=50)

    entrances = I1.exits[d_in]
    exits = I2.entrances[d_out]

    idxs_in = Set(k for (k,v) in entrances)
    idxs_out = Set(k for (k,v) in exits)
    idxs_common = intersect(Set(idxs_out), Set(idxs_in))

    ref_pt_in = I1.refpoints[d_in][:out]
    exit_pts_in = I1.exit_pts[d_in]
    D_in = Dict()
    for idx in idxs_in
        D_in[idx] = dot(exit_pts_in[idx] - ref_pt_in, unitVec(I1.θ[d_in]-π/2))
    end

    ref_pt_out = I2.refpoints[d_out][:in]
    entrance_pts_out = I2.entrance_pts[d_out]
    D_out = Dict()
    for idx in idxs_out
        D_out[idx] = dot(entrance_pts_out[idx] - ref_pt_out, unitVec(I2.θ[d_out]-π/2))
    end

    S = ref_line.s[end]
    ds = mean(diff(ref_line.s))

    if S - 2 * (Δs₁ + Δs₂) < 1
        # construct 3 roads
        # SEGMENT 1 - STRAIGHT SECTION
        p₁ = 1
        p₂ = Int(round(Δs₁ / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_in, D_in, D_in, urbanMap, entrances)
        # SEGMENT 2 - FAN SECTION
        p₁ = p₂
        p₂ = Int(round((S-Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_out, D_in, D_out, urbanMap, connections)
        # SEGMENT 3 - STRAIGHT
        p₁ = p₂
        p₂ = length(ref_line.s)
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_out, idxs_out, D_out, D_out, urbanMap, connections; exits=exits);
    else
        # construct 5 roads: straight, fan in, straight, fan out, straight
        # SEGMENT 1 - STRAIGHT SECTION
        p₁ = 1
        p₂ = Int(round(Δs₁ / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_in, D_in, D_in, urbanMap, entrances)
        # SEGMENT 2 - FAN SECTION
        p₁ = p₂
        p₂ = Int(round((Δs₂+Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_in, idxs_common, D_in, D_in, urbanMap, connections)
        # SEGMENT 3 - STRAIGHT
        Δs₃ = S - 2 * (Δs₁ + Δs₂) # straight section length
        p₁ = p₂
        p₂ = Int(round((Δs₃+Δs₂+Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_common, idxs_common, D_in, D_out, urbanMap, connections)
        # SECTION 4 - FAN OUT
        Δs₄ = Δs₂ # fan out section length
        p₁ = p₂
        p₂ = Int(round((Δs₄+Δs₃+Δs₂+Δs₁) / ds))
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_common, idxs_out, D_out, D_out, urbanMap, connections)
        # SECTION 4 - STRAIGHT
        p₁ = p₂
        p₂ = length(ref_line.s)
        curve = Curve(ref_line.x[p₁:p₂],ref_line.y[p₁:p₂];s=ref_line.s[p₁:p₂],θ=ref_line.θ[p₁:p₂],k=ref_line.k[p₁:p₂])
        connections = ConstructRoadFromReference!(curve, idxs_out, idxs_out, D_out, D_out, urbanMap, connections; exits=exits);
    end
end

function GenerateDoubleRoadBetweenIntersections!(urbanMap, I1, I2, d_in, d_out; Δs₁=10, Δs₂=40, speed=50)
    """
    Generate connecting roadway in both directions
    """

    ref_pt1 = (I1.refpoints[d_in][:out] + I1.refpoints[Opposite[d_in]][:in]) / 2
    ref_pt2 = (I2.refpoints[Opposite[d_out]][:in] + I2.refpoints[d_out][:out]) / 2
    spline = SplineFromEndPoints(ref_pt1, ref_pt2, I1.θ[d_in], I2.θ[Opposite[d_out]]; numPts=200, Δs=1.0);
    ref_line = Curve(spline.x, spline.y; s=spline.s, θ=spline.θ, k=spline.k)
    lat_vec = hcat(cos.(ref_line.θ-π/2),sin.(ref_line.θ-π/2))
    XY = hcat(ref_line.x,ref_line.y)

    trans_spline_1 = TransitionSpline(
        length(ref_line.s);
        y₁=dot(I1.refpoints[d_in][:out] - ref_pt1, unitVec(I1.θ[d_in]-π/2)),
        y₂=dot(I2.refpoints[Opposite[d_out]][:in] - ref_pt2, unitVec(I2.θ[Opposite[d_out]]-π/2)),
        L=ref_line.s[end], B=0.5)
    trans_spline_2 = TransitionSpline(
        length(ref_line.s);
        y₁=dot(I1.refpoints[Opposite[d_in]][:in] - ref_pt1, unitVec(I1.θ[d_in]-π/2)),
        y₂=dot(I2.refpoints[d_out][:out] - ref_pt2, unitVec(I2.θ[Opposite[d_out]]-π/2)),
        L=ref_line.s[end], B=0.5);

    xy1 = XY .+ trans_spline_1 .* lat_vec
    x1 = xy1[:,1]
    y1 = xy1[:,2]
    s1,θ1,k1 = ComputeFrenetCoordsFromCartesian(x1,y1;θ₁=I1.θ[d_in],θ₂=I2.θ[Opposite[d_out]])
    ref_line1 = Curve(x1,y1;s=s1,θ=θ1,k=k1)

    xy2 = XY .+ trans_spline_2 .* lat_vec
    x2 = reverse(xy2[:,1])
    y2 = reverse(xy2[:,2])
    s2,θ2,k2 = ComputeFrenetCoordsFromCartesian(x2,y2;θ₁=I2.θ[d_out],θ₂=I1.θ[Opposite[d_in]])
    ref_line2 = Curve(x2,y2;s=s2,θ=θ2,k=k2)

    GenerateRoadBetweenIntersections!(urbanMap, I1, I2, d_in, Opposite[d_out], ref_line1; Δs₁=Δs₁, Δs₂=Δs₂, speed=speed)
    GenerateRoadBetweenIntersections!(
        urbanMap, I2, I1, d_out, Opposite[d_in], ref_line2; Δs₁=Δs₁, Δs₂=Δs₂, speed=speed)
end

end # module
