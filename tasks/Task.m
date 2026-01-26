classdef Task < handle

    properties
        xdotbar = [] % reference task velocity
        J = []       % task Jacobian
        A = []       % task internal activation function
        ap
        ID
        task_name
        smooth
    end

    methods (Abstract)
        updateReference(obj)
        updateJacobian(obj)
        updateActivation(obj)
    end
end