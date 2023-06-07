@testset "Downloader selection" begin
    @test LightOSM.osm_network_downloader(:place_name) == LightOSM.osm_network_from_place_name
    @test LightOSM.osm_network_downloader(:bbox) == LightOSM.osm_network_from_bbox
    @test LightOSM.osm_network_downloader(:point) == LightOSM.osm_network_from_point
    @test LightOSM.osm_network_downloader(:polygon) == LightOSM.osm_network_from_polygon
    @test_throws ArgumentError LightOSM.osm_network_downloader(:doesnt_exist)
end

function wait_for_overpass()
    count = 0
    while !LightOSM.is_overpass_server_availabile()
        count == 7 && break # Can't wait indefinitely, and the failure will be caught in the tests
        count += 1
        @info "Waiting for overpass server..."
        sleep(5 * count)
    end
end

@testset "Downloads" begin
    # filenames = ["map.osm", "map.json"]
    # formats = [:osm, :json]
    # extra_way_filters = Dict{String,Vector{String}}(
    #     "highway" => ["primary", "secondary", "tertiary", "living_street"]
    # )
    # custom_way_exclusion_filters = [
    #     nothing,
    #     merge(LightOSM.WAY_EXCLUSION_FILTERS[:drive], extra_way_filters)
    # ]
    filenames = ["map.json"]
    formats = [:json]
    extra_way_filters = Dict{String,Vector{String}}(
        "highway" => ["primary", "secondary", "tertiary", "living_street"]
    )
    custom_way_exclusion_filters = [
        Dict(    # Merge default :drive filters and extra_way_filters
            haskey(extra_way_filters,k) ? (k => collect(union(Set(v), Set(extra_way_filters[k])))) : (k => v) 
            for (k,v) in LightOSM.WAY_EXCLUSION_FILTERS[:drive]
        )
    ]
    for (filename, format, custom_way_exclusion_filter) in zip(filenames, formats, custom_way_exclusion_filters)
        try
            wait_for_overpass()
            data = download_osm_network(:point,
                                        radius=0.5,
                                        point=GeoLocation(-37.8136, 144.9631),
                                        network_type=:drive,
                                        download_format=format,
                                        save_to_file_location=filename,
                                        custom_way_exclusion_filters=custom_way_exclusion_filter);
            @test isfile(filename)
            g = graph_from_file(filename, custom_way_exclusion_filters=custom_way_exclusion_filter) # Check it doesn't error

            # Check if custom way exclusion filters worked
            isnothing(custom_way_exclusion_filter) && continue
            @info custom_way_exclusion_filter
            num_ways_with_excluded_tags_in_download = count(
                element -> count(
                    (tag_name, excluded_values) -> haskey(element.tags, tag_name) && element.tags[tag_name] in excluded_values,
                    custom_way_exclusion_filter
                ),
                data["elements"]
            )
            ways_dont_have_excluded_tags_in_download = true
            for element in data["elements"]
                (element["type"] != "way") && continue
                for (tag_name, tag_value) in element["tags"]
                    if haskey(custom_way_exclusion_filter, tag_name) && (tag_value in custom_way_exclusion_filter[tag_name])
                        @info element
                        ways_dont_have_excluded_tags_in_download = false
                        break
                    end
                end
            end
            @test ways_dont_have_excluded_tags_in_download
        catch err
            # Sometimes gets HTTP.ExceptionRequest.StatusError in tests due to connection to overpass
            !isa(err, HTTP.ExceptionRequest.StatusError) && rethrow()
            @error "Test failed due to connection issue" exception=(err, catch_backtrace())
        end

        try
            rm(filename)
        catch
        end
    end
end