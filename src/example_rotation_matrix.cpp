/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        
        Some basics about the 3D rigid body motion.

        There are three different use of rotation matrix.
        1) Represent an orientation.
        2) Rotate an vector or frame (set of vectors).
        3) Change reference frame.

        Any angular velocity in 3D can be represented by an axis and speed of rotation
        about that axis. 
        
        We can express the axis as unit-vector in s-frame thus if it
        is multiplied by the rate of rotation to get the angular velocities in s-frame
        e.g. if the b-frame is rotating about a unit vector expressed in s-frame with
        dtheta rotation rate then it's angular velocity in s-frame can be expressed as 
        w_s = (w_s/||w_s||) * dtheta

        Similarly the linear velocity for each of the unit vector of b-frame can be
        determined as follows (the head of each unit vector is tracing a circle about
        the reference axis, therefore, it's linear velocity is tangent to this circle)
        dx_b = w_s x x_b (where x_b is a unit-vector), similar relationship hold for
        other two axis.

        It can be written in a vector form using a matrix-bracket form
        v_b = |_ws_| * x_b
        => dR_sb = |_ws_| * R_sb (Means Rate of rotation of b wrt s is w_s times R_sb)
        w_s is called skey-symmetric if |_ws_| = -|_ws_|^T
        Using subscript cancellation
        w_b = R_bs * w_s = R_sb^-1 * w_s = R_sb^T * w_s
        Where R_sb is the rotation of b-frame wrt s-frame, by droping the index
        w_b = R^T * w_s => w_s = R*w_b
        |_wb_| = R^T * dR          
        |_ws_| = dR * R^T          

    Notes:
        1) Right-hand coordinate system, index finger points in positive x-direction,
           middle finger along positive y-direction and the thum along positive z-direction.
        2) The positive angle is in the direction of the fingers curl about the thum axis.
        3) R_sb means the orientation of body frame (b) with respect to the spatial frame (s),
           the first subscript is the reference frame, where the second is the body frame.
           Similarly, a vector can be described in a frame by specifying the frame in which it
           is defined such as v_b.
        4) The reference and body frame for a series of transformation can be obtained by the
           subscript cancellation rule e.g.
           R_sb = R_sx * R_xy * R_yz * R_zb
           The subscript cancellation rule can be defined as follows:
           if the second subscript of the first matrix matches the first subscript of the second
           matrix they cancel each other.
        5) By pre-multiplying a rotation matrix by another rotation matrix we change reference of 
           of one frame into the second e.g.
           R_sc = R_sc * R_cb
           In this example by pre-multiplying R_cb with R_sc, we change the referenc of b-frame
           from c-frame to s-frame.
           Similarly by pre-multiplying a vector with a matrix, we change the referece frame of 
           the vector into the first subscript of the matrix, e.g.
           v_s = R_sb * v_b
        6) Finally, as the rotation matrix can also represent a rotation about an axis e.g.
           R = R(z, 90)
           So pre-multiplying a Rotation matrix (R) to a transformation matrix (R_sb) is 
           interpreted as the z-axis of the first subscript (reference frame) e.g.
           R*R_sb = R(z, 90)*R_sb means the rotation of b-frame about the z-axis of the s-frame.
           Similarly, by post-multiplying means the rotation about the z-axis of the second 
           subscirpt (body frame) e.g. 
           R_sb * R = R_sb * R(z, 90) means the rotation of b-frame about the z-axis of the b-frame.
        7) The angular velocity or rate of rotation matrix can be expressed by 3-parameters instead
           of 9 due to the fact that the velocity is tangent to the sphere in 3D. Thus
           w_s = R_sb * w_b
           w_b = R_bs * w_s = R_sb^T * w_s
           Where R_sb is the rotation of b-frame wrt s-frame
           |_wb_| = R_sb^T * dR
         8)  
           
*/