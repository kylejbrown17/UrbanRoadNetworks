module UrbanNetworkDemos

using RoadwayNetworks

export
    urbanDemo1,
    urbanDemo2

function urbanDemo1()
    urbanMap = UrbanMap();

    LanesIn1 = Dict(
        :North => [Set([:Left]), Set([:Left]), Set([:Left,:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :East => [Set([:Left]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :South => [Set([:Left,:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :West => [Set([:Left]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])]
    );
    LanesIn2 = Dict(
        :North => [Set([:Left]), Set([:Left,:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :East => [Set([:Left]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :South => [Set([:Left,:Straight]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :West => [Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])]
    );
    LanesIn3 = LanesIn2
    LanesIn4 = LanesIn2;

    id = getId!(urbanMap.dispenser); x = 0; y = 0; θ₁ = -π/6; θ₂ = θ₁+π/2;
    I1 = Intersection(id,x,y,θ₁,θ₂; LanesIn=LanesIn1); urbanMap.intersections[id] = I1;
    id = getId!(urbanMap.dispenser); x = 210; y = 2; θ₁ = π/6; θ₂ = θ₁+π/2;
    I2 = Intersection(id,x,y,θ₁,θ₂; LanesIn=LanesIn2); urbanMap.intersections[id] = I2;
    id = getId!(urbanMap.dispenser); x = 30; y = 130; θ₁ = 0; θ₂ = θ₁+π/2.5;
    I3 = Intersection(id,x,y,θ₁,θ₂; LanesIn=LanesIn3); urbanMap.intersections[id] = I3;
    id = getId!(urbanMap.dispenser); x = 180; y = 160; θ₁ = -π/6; θ₂ = θ₁+π/2;
    I4 = Intersection(id,x,y,θ₁,θ₂; LanesIn=LanesIn4); urbanMap.intersections[id] = I4;

    populateIntersection!(I1, urbanMap; LanesIn=LanesIn1, width=2.5, buffer=5)
    populateIntersection!(I2, urbanMap; LanesIn=LanesIn2, width=2.5, buffer=6);
    populateIntersection!(I3, urbanMap; LanesIn=LanesIn3, width=2.5, buffer=6);
    populateIntersection!(I4, urbanMap; LanesIn=LanesIn4, width=2.5, buffer=6);

    GenerateDoubleRoadBetweenIntersections!(urbanMap, I1, I2, :East, :West);
    GenerateDoubleRoadBetweenIntersections!(urbanMap, I1, I2, :South, :South);
    GenerateDoubleRoadBetweenIntersections!(urbanMap, I2, I4, :North, :South);
    GenerateDoubleRoadBetweenIntersections!(urbanMap, I2, I4, :East, :East);
    GenerateDoubleRoadBetweenIntersections!(urbanMap, I1, I3, :North, :South);
    GenerateDoubleRoadBetweenIntersections!(urbanMap, I1, I3, :West, :West);
    GenerateDoubleRoadBetweenIntersections!(urbanMap, I3, I4, :East, :West);
    GenerateDoubleRoadBetweenIntersections!(urbanMap, I3, I4, :North, :North);

    return urbanMap
end

# Demo
function urbanDemo2()
    LanesIn1 = Dict(
        :North => [Set([:Left]), Set([:Left]), Set([:Left,:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :East => [Set([:Left]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :South => [Set([:Left,:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :West => [Set([:Left]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])]
    );
    LanesIn2 = Dict(
        :North => [Set([:Left]), Set([:Left,:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :East => [Set([:Left]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :South => [Set([:Left,:Straight]), Set([:Straight]), Set([:Straight,:Right])],
        :West => [Set([:Left]), Set([:Straight]), Set([:Straight]), Set([:Straight,:Right])]
    );

    urbanMap = UrbanMap()

    θ = collect(linspace(-π/8,π/8,4))
    radii = collect(linspace(400,900,4))
    id_dict = Dict()
    for (i,θp) in enumerate(θ)
        for (j,r) in enumerate(radii)
            x = 10*rand()-5 + r*cos(θp)
            y = 10*rand()-5 + r*sin(θp)
            θ₁ = θp + rand()*π/6 - π/12
            θ₂ = θ₁ + π/2 + rand()*π/3 - π/6
            id = getId!(urbanMap)
            if rand() > 0.5
                intersection = Intersection(id,x,y,θ₁,θ₂; LanesIn=LanesIn1)
                urbanMap.intersections[id] = intersection
            else
                intersection = Intersection(id,x,y,θ₁,θ₂; LanesIn=LanesIn2)
                urbanMap.intersections[id] = intersection
            end
            id_dict[(i,j)] = id
        end
    end
    for (id, intersection) in urbanMap.intersections
        populateIntersection!(intersection, urbanMap; width=2.5, buffer=5)
    end
    for i in 1:length(θ)
        for j in 1:length(radii)
            I1 = urbanMap.intersections[id_dict[(i,j)]]
            if i < length(θ)
                IN = urbanMap.intersections[id_dict[(i+1,j)]]
                GenerateRoadBetweenIntersections!(urbanMap, I1, IN, :North, :North);
            end
            if i > 1
                IS = urbanMap.intersections[id_dict[(i-1,j)]]
                GenerateRoadBetweenIntersections!(urbanMap, I1, IS, :South, :South);
            end
            if j < length(radii)
                IE = urbanMap.intersections[id_dict[(i,j+1)]]
                GenerateRoadBetweenIntersections!(urbanMap, I1, IE, :East, :East);
            end
            if j > 1
                IW = urbanMap.intersections[id_dict[(i,j-1)]]
                GenerateRoadBetweenIntersections!(urbanMap, I1, IW, :West, :West);
            end
        end
    end
    return urbanMap
end

end
