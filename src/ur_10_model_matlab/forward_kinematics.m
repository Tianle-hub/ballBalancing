function H_stack = forward_kinematics(DH)
    dof = size(DH, 1);
    H_stack = sym(zeros(4,4,dof + 1));
    H_stack(:,:,1) = sym(eye(4));
    
    for i = 2 : dof + 1
        H_stack(:,:,i) = H_stack(:,:,i-1)*relativeTrans(DH(i-1,:));
    end
    
end

