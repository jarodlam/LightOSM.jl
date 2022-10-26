using ProgressLogging
using TerminalLoggers
using Logging

macro _withprogress(name, exp)
    quote
        if !eval(verbose)
            $exp
        else
            Base.with_logger(TerminalLogger()) do
                @withprogress name=$name begin
                    $exp
                end
            end
        end
    end |> esc
end

macro _logprogress(args...)
    quote
        eval(verbose) && @logprogress($(args...))
    end |> esc
end

macro _function_withprogress(name, exp)
    # exp: block containing function definition
    # exp.args[2]: function definition
    # exp.args[2].args[2]: function body
    func_body = ex.args[2].args[2]

    func_body = quote
        if !eval(verbose)
            $func_body
        else
            Base.with_logger(TerminalLogger()) do
                @withprogress name=$name begin
                    $func_body
                end
            end
        end
    end |> esc

    return exp
end

function subprocess(;parent_logger=nothing)
    @progress for i=1:10 parentid=parent_logger
        @info @progressid
        sleep(0.1)
    end
end

# with_logger(TerminalLogger()) do
#     @progress "Generating graph" for i=1:10 
#         @info @progressid
#         subprocess(parent_logger=@progressid)
#     end
# end

# verbose = false
# @_withprogress "Generating graph" begin
#     @logprogress 0
#     sleep(1)
#     @logprogress 0.5
#     sleep(1)
#     @logprogress 1
# end

verbose = true
@_withprogress "Generating graph" begin
    @_logprogress 0
    sleep(1)
    @_logprogress 0.5
    sleep(1)
    @_logprogress 1
end
