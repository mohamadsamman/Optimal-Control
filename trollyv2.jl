using JuMP
using Ipopt
using Plots

 

# Initialize the JuMP model
sys = Model(optimizer_with_attributes(Ipopt.Optimizer,"print_level"=> 5))
set_optimizer_attribute(sys,"tol",1e-6)
set_optimizer_attribute(sys,"max_iter",1000)

# Parameters
x0 = 10 
v0 = 10 
xf = 0
vf = 0
P = 100

# Bounds for variables

@variables(sys,begin
    q[2:P]           # x , v
    -1 ≤ u[1:P] ≤ 1  # u, control
     0 ≤ Δt ≤ 1 
    end)

# Objective
@objective(sys,Min,Δt)

# Constraints 
@constraints(sys,begin
    q[1,1] == x0
    q[2,1] == v0
    q[1,P] == xf
    q[2,P] == vf
    end)

#Crank-Nicolson
for j in 1 : P-1
    @NLconstraint(sys, # x' = w + cos(theta)
        q[1,j+1] == q[1,j] + Δt / 2 * (q[2,j] + q[2,j+1]))
    @NLconstraint(sys, # y' = sin(theta) 
        q[2,j+1] == q[2,j] + Δt / 2 * (u[j] + u[j+1]))
end

# Solves for the control and state
println("Solving...")
status = optimize!(sys)
println("Solver status : ",status)
x1 = value.(q[1,:])
v1 = value.(q[2,:])
u1 = value.(u)
println("Cost : " , objective_value(sys))
println("tf = ", value.(Δt)*P)

# Plots: states 
Δt1 = value.(Δt)
t = (1 : P)*Δt1
x_plot = plot(t,x1,xlabel = "t", ylabel = "position x", legend = false, fmt = :png)
v_plot = plot(t,v1,xlabel = "t", ylabel = "vitesse v", legend = false, fmt = :png)
u_plot = plot(t,u1,xlabel = "t", ylabel = "control", legend = false, fmt = :png)
display(plot(x_plot,v_plot,u_plot, layout = (2,2)))


current()