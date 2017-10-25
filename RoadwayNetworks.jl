module RoadwayNetworks

using SplineUtils
using GeomUtils

export
    Curve,
    curvePt,
    Lane,
    Road,
    Intersection,
    UrbanMap,
    id_dispenser,
    getId!

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
    predecessors # other lanes
    speed
    function Road(id;curve=nothing,speed=nothing)
        road = new()
        road.id = id
        road.referenceLine = curve
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
    function Lane(lane_id,x,y,s,θ,k;width=nothing,road_id=nothing)
        lane = new()
        lane.id = lane_id
        lane.road = road_id
        lane.centerLine = Curve(x,y;s=s,θ=θ,k=k)
        lane.length = lane.centerLine.s[end]
        lane.width = width
        lane.boundaryLeft = nothing
        lane.boundaryRight = nothing
        lane.predecessors = nothing
        lane.successors = nothing
        lane.neighborsLeft = nothing
        lane.neighborsRight = nothing
        return lane
    end
end

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
    lanes # Dict(id) => Lane
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
        retval.entrances = Dict(
            :North => Dict(),
            :South => Dict(),
            :East => Dict(),
            :West => Dict()
        )
        retval.exits = Dict(
            :North => Dict(),
            :South => Dict(),
            :East => Dict(),
            :West => Dict()
        )
        retval.refpoints = Dict(
            :North => Dict(),
            :South => Dict(),
            :East => Dict(),
            :West => Dict()
        )
        retval.lanes = Dict(
            :North => Dict(),
            :South => Dict(),
            :East => Dict(),
            :West => Dict()
        )
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
    function UrbanMap(;numElements=1000000)
        urbanMap = new()
        urbanMap.roads = Dict()
        urbanMap.lanes = Dict()
        urbanMap.intersections = Dict()
        urbanMap.points = Dict()
        urbanMap.dispenser = id_dispenser()
        return urbanMap
    end
end

end # module
