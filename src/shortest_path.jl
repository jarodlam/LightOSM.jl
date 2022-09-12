"""
    shortest_path([PathAlgorithm,]
                  g::OSMGraph,
                  origin::Union{Integer,Node},
                  destination::Union{Integer,Node},
                  [weights::AbstractMatrix=g.weights;
                  cost_adjustment::Function=(u, v, parents) -> 0.0),
                  max_distance::W=typemax(W)]

Calculates the shortest path between two OpenStreetMap node ids.

# Arguments
- `PathAlgorithm` (optional): Path finding algorithm, possible values are:
    - `Dijkstra`: Same as `DijkstraVector`. This is the default algorithm.
    - `DijkstraVector`: Dijkstra's algorithm using the `Vector` implementation. 
        Faster for small graphs and/or long paths.
    - `DijkstraDict`: Dijkstra's algorithm using the `Dict` implementation.
        Faster for large graphs and/or short paths.
    - `AStar`: Same as `AStarVector`.
    - `AStarVector`: A* algorithm using the `Vector` implementation.
        Faster for small graphs and/or long paths.
    - `AStarDict`: A* algorithm using the `Dict` implementation.
        Faster for large graphs and/or short paths.
- `g::OSMGraph{U,T,W}`: Graph container.
- `origin::Union{Integer,Node}`: Origin OpenStreetMap node or node id.
- `destination::Union{Integer,Node},`: Destination OpenStreetMap node or node 
    id.
- `weights`: Optional matrix of node to node edge weights, defaults to 
    `g.weights`. If a custom weights matrix is being used with algorithm set to
    `AStar`, make sure that a correct heuristic is being used.
- `cost_adjustment::Function=(u,v,parents) -> 0.0`: Option to pass in a function 
    to adjust the cost between each pair of vetices `u` and `v`, normally the 
    cost is just the weight between `u` and `v`, `cost_adjustment` takes in 3 
    arguments; `u`, `v` and `parents`; to apply an additive cost to the default 
    weight. Defaults no adjustment. Use `restriction_cost_adjustment` to 
    consider turn restrictions.
- `heuristic::Function=distance_heuristic(g)`: Use custom heuristic with the 
    `AStar` algorithms only. Defaults to a function 
    `h(u, v) -> haversine(u, v)`, i.e. returns the haversine distances between 
    `u`, the current node, and `v`, the neighbouring node. If `g.weight_type` 
    is `:time` or `:lane_efficiency`, use `time_heuristic(g)` instead.

# Return
- `Union{Nothing,Vector{T}}`: Array of OpenStreetMap node ids making up 
    the shortest path.
"""
function shortest_path(::Type{A},
                       g::OSMGraph{U,T,W},
                       origin::Integer,
                       destination::Integer,
                       weights::AbstractMatrix{W};
                       cost_adjustment::Function=(u, v, parents) -> 0.0,
                       max_distance::W=typemax(W)
                       )::Union{Nothing,Vector{T}} where {A <: Dijkstra, U, T, W}
    o_index = node_id_to_index(g, origin)
    d_index = node_id_to_index(g, destination)
    path = dijkstra(A, g.graph, weights, o_index, d_index; cost_adjustment=cost_adjustment, max_distance=max_distance)
    isnothing(path) && return
    return index_to_node_id(g, path)
end
function shortest_path(::Type{A},
                       g::OSMGraph{U,T,W},
                       origin::Integer,
                       destination::Integer,
                       weights::AbstractMatrix{W};
                       cost_adjustment::Function=(u, v, parents) -> 0.0,
                       heuristic::Function=distance_heuristic(g),
                       max_distance::W=typemax(W)
                       )::Union{Nothing,Vector{T}} where {A <: AStar, U, T, W}
    o_index = node_id_to_index(g, origin)
    d_index = node_id_to_index(g, destination)
    path = astar(A, g.graph, weights, o_index, d_index; cost_adjustment=cost_adjustment, heuristic=heuristic, max_distance=max_distance)
    isnothing(path) && return
    return index_to_node_id(g, path)
end
function shortest_path(::Type{A}, g::OSMGraph{U,T,W}, origin::Integer, destination::Integer; kwargs...)::Union{Nothing,Vector{T}} where {A <: PathAlgorithm, U, T, W}
    return shortest_path(A, g, origin, destination, g.weights; kwargs...)
end
function shortest_path(::Type{A}, g::OSMGraph{U,T,W}, origin::Node{<:Integer}, destination::Node{<:Integer}, args...; kwargs...)::Union{Nothing,Vector{T}} where {A <: PathAlgorithm, U, T, W}
    return shortest_path(A, g, origin.id, destination.id, args...; kwargs...)
end
function shortest_path(g::OSMGraph{U,T,W}, args...;  kwargs...)::Union{Nothing,Vector{T}} where {U, T, W}
    return shortest_path(Dijkstra, g, args...; kwargs...)
end

"""
    shortest_path([::Type{<:PathAlgorithm}]
                  g::OSMGraph{U,T,W},
                  u::EdgePoint{T}, 
                  v::EdgePoint{T}; 
                  max_distance::W=typemax(W)
                  )::Union{Nothing,Vector{T}} where {U, T, W}

Finds the shortest path between two `EdgePoint`s.

# Arguments
- `::Type{<:PathAlgorithm}`: Kept for compatibility, algorithm is always `DijkstraDict`.
- `g::OSMGraph{U,T,W}`: LightOSM graph.
- `u::EdgePoint{T}`: Start point.
- `v::EdgePoint{T}`: Goal point.
- `max_distance::W=typemax(W)`: Maximum distance to search before giving up.

# Returns
- `::Vector{T}`: Node path between `EdgePoint`s.
- `::Nothing`: If no path was found.
"""
function shortest_path(g::OSMGraph{U,T,W},
                       u::EdgePoint{T}, 
                       v::EdgePoint{T}; 
                       max_distance::W=typemax(W)
                       )::Union{Nothing,Vector{T}} where {U, T, W}
    # Check if u and v are on the same edge
    same_edge_forward = u.n1 == v.n1 && u.n2 == v.n2
    same_edge_reverse = u.n1 == v.n2 && u.n2 == v.n1
    if same_edge_forward || same_edge_reverse
        # Check which edge point comes first
        if (same_edge_forward && u.pos < v.pos) ||
           (same_edge_reverse && u.pos < (1-v.pos))
            return [u.n1, u.n2]
        # Make sure an edge exists in the opposite direction
        elseif g.weights[g.node_to_index[u.n2], g.node_to_index[u.n1]] > 0.0
            return [u.n2, u.n1]
        # Can't travel in opposite direction, no convenient path possible
        else
            return nothing
        end
    end

    # Find shortest path using modified graph
    modified_graph, modified_weights, start_idx, goal_idx = construct_modified_graph(g, u, v)
    path = LightOSM.dijkstra(
        DijkstraDict, 
        modified_graph, 
        modified_weights, 
        start_idx, 
        goal_idx, 
        max_distance=max_distance
    )
    isnothing(path) && return nothing

    # Convert to nodes; convert start and goal nodes to their original nodes
    if !has_vertex(g.graph, path[1])
        n1_idx = g.node_to_index[u.n1]
        n2_idx = g.node_to_index[u.n2]
        path[1] = n2_idx == path[2] ? n1_idx : n2_idx
    end
    if !has_vertex(g.graph, path[end])
        n1_idx = g.node_to_index[v.n1]
        n2_idx = g.node_to_index[v.n2]
        path[end] = n2_idx == path[end-1] ? n1_idx : n2_idx
    end
    nodes = [g.index_to_node[x] for x in path]
    return nodes
end
function shortest_path(::Type{<:PathAlgorithm}, 
                       g::OSMGraph{U,T,W}, 
                       u::EdgePoint{T}, 
                       v::EdgePoint{T}; 
                       max_distance::W=typemax(W)
                       )::Union{Nothing,Vector{T}} where {U, T, W}
    return shortest_path(g, u, v, max_distance=max_distance)
end

"""
    set_dijkstra_state!(g::OSMGraph, src::Union{Integer,Vecotr{<:Integer}, weights::AbstractMatrix; cost_adjustment::Function=(u, v, parents) -> 0.0)

Compute and set the dijkstra parent states for one or multiple src vertices. Threads are used for multiple srcs.
Note, computing dijkstra states for all vertices is a O(V² + ElogV) operation, use on large graphs with caution.
"""
function set_dijkstra_state!(g::OSMGraph, src::Integer, weights::AbstractMatrix; cost_adjustment::Function=(u, v, parents) -> 0.0)
    g.dijkstra_states[src] = dijkstra(g.graph, weights, src; cost_adjustment=cost_adjustment)
end
function set_dijkstra_state!(g::OSMGraph, srcs::Vector{<:Integer}, weights::AbstractMatrix; cost_adjustment::Function=(u, v, parents) -> 0.0)
    Threads.@threads for src in srcs
        set_dijkstra_state!(g, src, weights; cost_adjustment=cost_adjustment)
    end
    return g
end
set_dijkstra_state!(g::OSMGraph, src; kwargs...) = set_dijkstra_state!(g, src, g.weights; kwargs...)

"""
    shortest_path_from_dijkstra_state(g::OSMGraph, origin::Integer, destination::Integer)

Extract shortest path from precomputed dijkstra state, from `origin` to `detination` node id.

Note, function will raise `UndefRefError: access to undefined reference` if the dijkstra state of the
origin node is not precomputed.

# Arguments
- `g::OSMGraph`: Graph container.
- `origin::Integer`: Origin OpenStreetMap node or node id.
- `destination::Integer`: Destination OpenStreetMap node or node id.

# Return
- `Union{Nothing,Vector{T}}`: Array of OpenStreetMap node ids making up the shortest path.
"""
function shortest_path_from_dijkstra_state(g::OSMGraph, origin::Integer, destination::Integer)
    parents = node_id_to_dijkstra_state(g, origin)
    path = path_from_parents(parents, node_id_to_index(g, destination))
    isnothing(path) && return
    return index_to_node_id(g, path)
end 

"""
    is_restricted(restriction_ll::MutableLinkedList{V}, u::U, v::U, parents::P)::Bool where {P <: Union{<:AbstractVector{<:U}, <:AbstractDict{<:U, <:U}}} where {U <: Integer,V <: Integer}

Given parents, returns `true` if path between `u` and `v` is restricted by the restriction linked list, `false` otherwise.

# Arguments
- `restriction_ll::MutableLinkedList{V}`: Linked list holding vertices in order of v -> parents.
- `u::U`: Current vertex visiting.
- `v::U`: Current neighbour vertex.
- `parents::Union{<:AbstractVector{<:U}, <:AbstractDict{<:U, <:U}}`: Mapping of shortest path children to parents.

# Return
- `Bool`: Returns true if path between `u` and `v` is restricted.
"""
function is_restricted(restriction_ll::MutableLinkedList{V}, u::U, v::U, parents::P)::Bool where {P <: Union{<:AbstractVector{<:U}, <:AbstractDict{<:U, <:U}}} where {U <: Integer,V <: Integer}
    current = restriction_ll.node.next

    if v != current.data
        return false
    end

    checked = 1 # already checked v

    while checked < restriction_ll.len
        current = current.next

        if u == current.data
            u = get(parents, u, zero(U))
        else
            return false
        end

        checked += 1
    end

    return true
end

"""
    restriction_cost(restrictions::AbstractDict{V,Vector{MutableLinkedList{V}}}, u::U, v::U, parents::Vector{U})::Float64 where {U <: Integer,V <: Integer}

Given parents, returns `Inf64` if path between `u` and `v` is restricted by the set of restriction linked lists, `0.0` otherwise.

# Arguments
- `restrictions::AbstractDict{V,Vector{MutableLinkedList{V}}}`: Set of linked lists holding vertices in order of v -> parents.
- `u::U`: Current vertex visiting.
- `v::U`: Current neighbour vertex.
- `parents::Union{<:AbstractVector{<:U}, <:AbstractDict{<:U, <:U}}`: Mapping of shortest path children to parents.

# Return
- `Float64`: Returns `Inf64` if path between u and v is restricted, `0.0` otherwise.
"""
function restriction_cost(restrictions::AbstractDict{V,Vector{MutableLinkedList{V}}}, u::U, v::U, parents::P)::Float64 where {P <: Union{<:AbstractVector{<:U}, <:AbstractDict{<:U, <:U}}} where {U <: Integer,V <: Integer}
    !haskey(restrictions, u) && return 0.0

    for ll in restrictions[u]
        is_restricted(ll, u, v, parents) && return typemax(Float64)
    end

    return 0.0
end

"""
    restriction_cost_adjustment(g::OSMGraph)

Returns the cost adjustment function (user in dijkstra and astar) for restrictions. The return function 
takes 3 arguments, `u` being the current node, `v` being the neighbour node, `parents` being the array 
of parent dijkstra states. By default `g.indexed_restrictions` is used to check whether the path from 
`u` to `v` is restricted given all previous nodes in `parents`.
"""
restriction_cost_adjustment(g::OSMGraph) = (u, v, parents) -> restriction_cost(g.indexed_restrictions, u, v, parents)

"""
    distance_heuristic(g::OSMGraph)

Returns the heuristic function used in astar shortest path calculation, should be used with a graph with
`weight_type=:distance`. The heuristic function takes in 2 arguments, `u` being the current node and `v` 
being the neighbour node, and returns the haversine distance between them.
"""
distance_heuristic(g::OSMGraph) = (u, v) -> haversine(g.node_coordinates[u], g.node_coordinates[v])

"""
    time_heuristic(g::OSMGraph)

Returns the heuristic function used in astar shortest path calculation, should be used with a graph with
`weight_type=:time` or `weight_type=:lane_efficiency`. The heuristic function takes in 2 arguments, `u` 
being the current node and `v` being the neighbour node, and returns the estimated travel time between them. 
Calculated by dividing the harversine distance by a fixed maxspeed of `100`. Remember to achieve an optimal
path, it is important to pick an *underestimating* heuristic that best estimates the cost remaining to the `goal`,
hence we pick the largest maxspeed across all ways.
"""
time_heuristic(g::OSMGraph) = (u, v) -> haversine(g.node_coordinates[u], g.node_coordinates[v]) / 100.0

"""
    weights_from_path(g::OSMGraph{U,T,W}, path::Vector{T}; weights=g.weights)::Vector{W} where {U <: Integer,T <: Integer,W <: Real}

Extracts edge weights from a path using the weight matrix stored in `g.weights` unless
a different matrix is passed to the `weights` kwarg.

# Arguments
- `g::OSMGraph`: Graph container.
- `path::Vector{T}`: Array of OpenStreetMap node ids.
- `weights=g.weights`: the matrix that the edge weights are extracted from. Defaults to `g.weights`.

# Return
- `Vector{W}`: Array of edge weights, distances are in km, time is in hours.
"""
function weights_from_path(g::OSMGraph{U,T,W}, path::Vector{T}; weights=g.weights)::Vector{W} where {U <: Integer,T <: Integer,W <: Real}
    return [weights[g.node_to_index[path[i]], g.node_to_index[path[i + 1]]] for i in 1:length(path) - 1]
end

"""
    total_path_weight(g::OSMGraph{U,T,W}, path::Vector{T}; weights=g.weights)::W where {U <: Integer,T <: Integer,W <: Real}

Extract total edge weight along a path.

# Arguments
- `g::OSMGraph`: Graph container.
- `path::Vector{T}`: Array of OpenStreetMap node ids.
- `weights=g.weights`: the matrix that the edge weights are extracted from. Defaults to `g.weights`.

# Return
- `sum::W`: Total path edge weight, distances are in km, time is in hours.
"""
function total_path_weight(g::OSMGraph{U,T,W}, path::Vector{T}; weights=g.weights)::W where {U <: Integer,T <: Integer,W <: Real}
    sum::W = zero(W)
    for i in 1:length(path) - 1
        sum += weights[g.node_to_index[path[i]], g.node_to_index[path[i + 1]]]
    end
    return sum
end

"""
    construct_modified_graph(g::OSMGraph{U,T,W},
                             u::EdgePoint{T}, 
                             v::EdgePoint{T}
                             ) where {U, T, W}

Constructs `ModifedGraph` and `ModifiedWeights` objects suitable for finding the shortest 
path between two `EdgePoint`s.

# Arguments
- `g::OSMGraph{U,T,W}`: LightOSM graph.
- `u::EdgePoint{T}`: Start point.
- `v::EdgePoint{T}`: Goal point.

# Returns
- `::Tuple`:
  - `::ModifiedGraph`: Graph suitable for finding shortest path.
  - `::ModifiedWeights`: Weights matrix suitable for finding shortest path.
  - `::U`: Starting graph index.
  - `::V`: Goal graph index.
"""
function construct_modified_graph(g::OSMGraph{U,T,W},
                                  u::EdgePoint{T}, 
                                  v::EdgePoint{T}
                                  ) where {U, T, W}
    # Pre-allocate memory
    start_idx = nv(g.graph) + U(1)
    goal_idx = nv(g.graph) + U(2)
    edges_add = Dict{U,Set{U}}(
        g.node_to_index[u.n1] => Set{U}(),
        g.node_to_index[u.n2] => Set{U}(),
        g.node_to_index[v.n1] => Set{U}(),
        g.node_to_index[v.n2] => Set{U}(),
        start_idx => Set{U}(),
        goal_idx => Set{U}()
    )
    edges_rm = Dict{U,Set{U}}(
        g.node_to_index[u.n1] => Set{U}(),
        g.node_to_index[u.n2] => Set{U}(),
        g.node_to_index[v.n1] => Set{U}(),
        g.node_to_index[v.n2] => Set{U}()
    )
    weights_add = Dict{Tuple{U,U},W}()
    weights_rm = Set{Tuple{U,U}}()

    # Check for any EdgePoints that are actually directly on a node
    to_add = Tuple{EdgePoint{T},U}[]
    if u.pos <= zero(W)
        start_idx = g.node_to_index[u.n1]
    elseif u.pos >= one(W)
        start_idx = g.node_to_index[u.n2]
    else
        push!(to_add, (u, start_idx))
    end
    if v.pos <= zero(W)
        goal_idx = g.node_to_index[v.n1]
    elseif v.pos >= one(W)
        goal_idx = g.node_to_index[v.n2]
    else
        push!(to_add, (v, goal_idx))
    end

    # Populate the "graph"
    for (ep, ep_idx) in to_add
        # Node IDs to graph indices
        n1_idx = g.node_to_index[ep.n1]
        n2_idx = g.node_to_index[ep.n2]

        # Get edge weights for both directions
        w1 = g.weights[n1_idx, n2_idx]
        w2 = g.weights[n2_idx, n1_idx]

        # Check if the edge n1 -> n2 exists
        if w1 > zero(U)
            # Add n1 -> ep
            push!(edges_add[n1_idx], ep_idx)
            weights_add[(n1_idx, ep_idx)] = ep.pos * w1

            # Add ep -> n2
            push!(edges_add[ep_idx], n2_idx)
            weights_add[(ep_idx, n2_idx)] = (1 - ep.pos) * w1

            # Remove n1 -> n2
            push!(edges_rm[n1_idx], n2_idx)
            push!(weights_rm, (n1_idx, n2_idx))
        end
        # Check if the edge n2 -> n1 exists
        if w2 > zero(U) 
            # Add n2 -> ep
            push!(edges_add[n2_idx], ep_idx)
            weights_add[(n2_idx, ep_idx)] = (1 - ep.pos) * w2

            # Add ep -> n1
            push!(edges_add[ep_idx], n1_idx)
            weights_add[(ep_idx, n1_idx)] = ep.pos * w1

            # Remove n2 -> n1
            push!(edges_rm[n2_idx], n1_idx)
            push!(weights_rm, (n2_idx, n1_idx))
        end
    end

    mg = ModifiedGraph(
        g.graph,
        nv(g.graph) + U(2),
        edges_add,
        edges_rm
    )
    mw = ModifiedWeights(
        g.weights,
        nv(g.graph) + U(2),
        weights_add,
        weights_rm
    )
    return mg, mw, start_idx, goal_idx
end

"""
    shortest_path_distance(g::OSMGraph, 
                           u::EdgePoint, 
                           v::EdgePoint; 
                           max_distance::W=typemax(W)
                           )::Union{Nothing,W} where {U, T, W}

Finds the length of the shortest path between two `EdgePoint`s.

# Arguments
- `g::OSMGraph{U,T,W}`: LightOSM graph.
- `u::EdgePoint{<:Integer}`: Start point.
- `v::EdgePoint{<:Integer}`: Goal point.
- `max_distance::W=typemax(W)`: Maximum distance to search before giving up.

# Returns
- `::W`: Distance of shortest path.
- `::Nothing`: If no path was found.
"""
function shortest_path_distance(::Type{A},
                                g::OSMGraph{U,T,W}, 
                                orig::EdgePoint{T}, 
                                dest::EdgePoint{T}; 
                                max_distance::W=typemax(W)
                                )::Union{Nothing,W} where {U, T, W}
    # Check if orig and dest are on the same edge
    same_edge = orig.n1 == dest.n1 && orig.n2 == dest.n2
    same_edge_flipped = orig.n1 == dest.n2 && orig.n2 == dest.n1
    weight_forward = g.weights[g.node_to_index[orig.n1], g.node_to_index[orig.n2]]
    weight_reverse = g.weights[g.node_to_index[orig.n2], g.node_to_index[orig.n1]]
    if same_edge || same_edge_flipped
        if same_edge && orig.pos <= dest.pos && weight_forward > 0.0
            #=
            ●----------o-----d----------●
            orig.n1               orig.n2
            dest.n1               dest.n2
            =#
            return abs(orig.pos - dest.pos) * weight_forward
        elseif same_edge_flipped && orig.pos <= (1-dest.pos) && weight_forward > 0.0
            #=
            ●----------o-----d----------●
            orig.n1               orig.n2
            dest.n2               dest.n1
            =#
            return abs(orig.pos - (1-dest.pos)) * weight_forward
        elseif same_edge && orig.pos > dest.pos && weight_reverse > 0.0
            #=
            ●----------d-----o----------●
            orig.n1               orig.n2
            dest.n1               dest.n2
            =#
            return abs(orig.pos - dest.pos) * weight_reverse
        elseif same_edge_flipped && orig.pos > (1-dest.pos) && weight_reverse > 0.0
            #=
            ●----------d-----o----------●
            orig.n1               orig.n2
            dest.n2               dest.n1
            =#
            return abs(orig.pos - (1-dest.pos)) * weight_reverse
        else
            # One-way edge and trying to travel in opposite direction
            return nothing
        end
    end

    modified_graph, modified_weights, start_idx, goal_idx = construct_modified_graph(g, u, v)
    path =  LightOSM.dijkstra(
        DijkstraDict, 
        modified_graph, 
        modified_weights, 
        start_idx, 
        goal_idx, 
        max_distance=max_distance
    )
    isnothing(path) && return nothing

    total_weight = zero(W)
    for (curr, next) in zip(path[1:end-1], path[2:end])
        total_weight += modified_weights[curr, next]
    end
    return total_weight
end
function shortest_path_distance(g::OSMGraph{U,T,W}, args...;  kwargs...) where {U, T, W}
    return shortest_path_distance(Dijkstra, g, args...; kwargs...)
end
