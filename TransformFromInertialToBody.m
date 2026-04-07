function bodyframe = TransformFromInertialToBody(inertial,bodyangles)

DCM = angle2dcm(bodyangles(1), bodyangles(2), bodyangles(3), 'ZYX');
bodyframe =   DCM * inertial;


end