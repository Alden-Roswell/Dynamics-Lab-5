<<<<<<< HEAD
function bodyframe = TransformFromInertialToBody(inertial,bodyangles)

DCM = angle2dcm(bodyangles(1), bodyangles(2), bodyangles(3), 'ZYX');
bodyframe =   DCM * inertial;


=======
function bodyframe = TransformFromInertialToBody(inertial,bodyangles)

DCM = angle2dcm(bodyangles(1), bodyangles(2), bodyangles(3), 'ZYX');
bodyframe =   DCM * inertial;


>>>>>>> ee3fe1432b21c1bcb50e83e78077de5a355df5d6
end