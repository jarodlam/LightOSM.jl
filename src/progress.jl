const PROG_TIME_INTERVAL_NS = 0.5 * 1e9    # 0.5 seconds

function _withprogress(f::Function, name::AbstractString; verbose=nothing, parentid=nothing)
    if (!isnothing(verbose) && verbose == false) || isnothing(verbose) && isnothing(parentid)
        return f(nothing)
    end

    Base.with_logger(TerminalLogger()) do
        return ProgressLogging.progress(f, name=name)
    end
end

function _logprogress(id, fraction; kwargs...)
    isnothing(id) && return
    @info ProgressLogging.Progress(id, fraction; kwargs...)
end

function next_step!(prog::ProgressState)
    prog.current_step += 1
    return prog
end
next_step!(prog::Nothing, args...; kwargs...) = nothing

function next_substep!(prog::ProgressState)
    prog.current_substep += 1
    return prog
end
function next_substep!(prog::ProgressState, num_substeps::Integer)
    prog.total_substeps = num_substeps
    prog.current_substep = 1
    return prog
end
next_substep!(prog::Nothing, args...; kwargs...) = nothing

function log_step(prog::ProgressState, percent::AbstractFloat)
    log_progress(
        prog, 
        (prog.current_step - 1) + percent / prog.total_steps
    )
end
log_step(prog::Nothing, args...; kwargs...) = nothing

function log_substep(prog::ProgressState, percent::AbstractFloat)
    log_progress(
        prog, 
        (
            (prog.current_step - 1) + 
            ((prog.current_substep - 1) + percent) / prog.total_substeps
        ) / prog.total_steps
    )
end
log_substep(prog::Nothing, args...; kwargs...) = nothing

function log_progress(prog::ProgressState, overall_percent::AbstractFloat)
    curr_time = time_ns()
    ((curr_time - prog.prev_time) < PROG_TIME_INTERVAL_NS) && return
    prog.prev_time = curr_time

    @info ProgressLogging.Progress(prog.id, overall_percent)
end
