using JuMP
using Ipopt
using Plots

 

# Initialize the JuMP model
sys = Model(optimizer_with_attributes(Ipopt.Optimizer,"print_level"=> 5))
set_optimizer_attribute(sys,"tol",1e-6)
set_optimizer_attribute(sys,"max_iter",1000)

# Parameters
x0 = -400.84 
v0 = 120
xf = 0
vf = 0
P = 100

# Bounds for variables

@variables(sys,begin
    x[1:P]           # x 
    v[1:P]           # y 
    -1 ≤ u[1:P] ≤ 1  # u, control
     0 ≤ Δt ≤ 1 
    end)

# Objective
@objective(sys,Min,Δt)

# Constraints 
@constraints(sys,begin
    x[1] == x0
    v[1] == v0
    x[P] == xf
    v[P] == vf
    end)

#Crank-Nicolson
for j in 1 : P-1
    @NLconstraint(sys, # x' = w + cos(theta)
        x[j+1] == x[j] + Δt / 2 * (v[j] + v[j+1]))
    @NLconstraint(sys, # y' = sin(theta) 
        v[j+1] == v[j] + Δt / 2 * (u[j] + u[j+1]))
end

# Solves for the control and state
println("Solving...")
status = optimize!(sys)
println("Solver status : ",status)
x1 = value.(x)
v1 = value.(v)
u1 = value.(u)
println("Cost : " , objective_value(sys))
println("tf = ", value.(Δt)*P)

# Plots: states 
Δt1 = value.(Δt)
t = (1 : P)*Δt1
x_plot1 = plot(t,x1,xlabel = "t", ylabel = "position x", legend = false, fmt = :png)
v_plot1 = plot(t,v1,xlabel = "t", ylabel = "vitesse v", legend = false, fmt = :png)
u_plot1 = plot(t,u1,xlabel = "t", ylabel = "control", legend = false, fmt = :png)
pos_plot = plot(x1,v1,xlabel = "position", ylabel = "vitesse", legend = false, fmt = :png)
display(plot(x_plot1,v_plot1,u_plot1,pos_plot, layout = (2,2)))



#current()