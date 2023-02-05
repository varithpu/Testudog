
#include "../include/testudog/testudog.h"

Eigen::Matrix<double, 3, 4> robot::Testudog::get_joint_angle(){
    return joint_angle;
}

Eigen::Matrix<double, 3, 4> robot::Testudog::global2local_legpos(Eigen::Matrix<double, 3, 4> legpos_global, 
    double x_global, double y_global, double z_global, double roll, double pitch, double yaw)
{    
    //transformation matrix global to body double 
    Eigen::Matrix4d T_global_body = euler2transf(x_global, y_global, z_global, roll, pitch, yaw);

    //transformation matrix global to front left leg origin
    Eigen::Matrix4d T_global_front_left_origin = T_global_body * euler2transf(body_width/2, -body_front, 0, 0, 0, 0);

    //transformation matrix global to front right leg origin
    Eigen::Matrix4d T_global_front_right_origin = T_global_body * euler2transf(-body_width/2, -body_front, 0, 0, 0, 0);

    //transformation matrix global to back left leg origin
    Eigen::Matrix4d T_global_back_left_origin = T_global_body * euler2transf(body_width/2, body_back, 0, 0, 0, 0);

    //transformation matrix global to back right leg origin
    Eigen::Matrix4d T_global_back_right_origin = T_global_body * euler2transf(-body_width/2, body_back, 0, 0, 0, 0);

    //convert leg pos to 4 legs x (4x1) vectors
    Eigen::Matrix4d legpos_global_matix;
    legpos_global_matix.block<3,4>(0,0) = legpos_global;
    for (int i = 0; i <= 3; i++){
        legpos_global_matix(3,i) = 1.0;
    } 

    // find local leg position
    Eigen::Matrix4d legpos_local;
    legpos_local.col(0) =  invtransf(T_global_front_left_origin)*legpos_global_matix.col(0);
    legpos_local.col(1) =  invtransf(T_global_front_right_origin)*legpos_global_matix.col(1);
    legpos_local.col(2) =  invtransf(T_global_back_left_origin)*legpos_global_matix.col(2);
    legpos_local.col(3) =  invtransf(T_global_back_right_origin)*legpos_global_matix.col(3);

    return legpos_local.block<3,4>(0,0);
}

Eigen::Matrix4d robot::Testudog::euler2transf(double x, double y, double z, double roll, double pitch, double yaw){

    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result(0,3) = x;
    result(1,3) = y;
    result(2,3) = z;

    //rotate roll
    Eigen::Matrix3d roll_transf = Eigen::Matrix3d::Identity();
    roll_transf(1,1) = cos(roll);
    roll_transf(1,2) = -sin(roll);
    roll_transf(2,1) = sin(roll);
    roll_transf(2,2) = cos(roll);
    
    //rotate pitch
    Eigen::Matrix3d pitch_transf = Eigen::Matrix3d::Identity();
    roll_transf(0,0) = cos(pitch);
    roll_transf(0,2) = sin(pitch);
    roll_transf(2,0) = -sin(pitch);
    roll_transf(2,2) = cos(pitch);

    //rotate yaw
    Eigen::Matrix3d yaw_transf = Eigen::Matrix3d::Identity();
    roll_transf(0,0) = cos(yaw);
    roll_transf(0,1) = -sin(yaw);
    roll_transf(1,0) = sin(yaw);
    roll_transf(1,1) = cos(yaw);

    //rotational matrix
    Eigen::Matrix3d rotational = roll_transf*pitch_transf*yaw_transf;

    result.block<3,3>(0,0) = rotational;

    return result;
}

Eigen::Matrix4d robot::Testudog::invtransf(Eigen::Matrix4d transf){

    Eigen::Matrix4d result = Eigen::Matrix<double,4,4>::Identity();
    result.block(0,0,3,3) = transf.block(0,0,3,3).transpose();
    result.block(0,3,3,1) = -transf.block(0,0,3,3).transpose()*transf.block(0,3,3,1);
    return result;
}

Eigen::Matrix<double, 3, 4> robot::Testudog::inv_kine(Eigen::Matrix<double, 3, 4> legpos_local){
    
    Eigen::Matrix<double, 3, 4> result;
    for (int i = 0; i <= 3; i++){
        double x = legpos_local(0,i);
        double y = legpos_local(1,i);
        double z = legpos_local(2,i);

        double F = sqrt(x*x + z*z - hip_length*hip_length);
        double hip_rolling_angle;

        if (i==1||i==2){
            hip_rolling_angle = atan2(z, -x*pow((-1),i)) + atan2(F, -hip_length);
        }else{
            hip_rolling_angle = -atan2(z, -x*pow((-1),i)) - atan2(F, -hip_length);
        }

        double D = (F*F + y*y - upperleg_length*upperleg_length - lowerleg_length*lowerleg_length) / (2*upperleg_length*lowerleg_length);
        
        double knee_angle = -atan2((sqrt(1-D*D)), D);

        double hip_pitching_angle = pow((-1),i)*(atan2(y,F) - atan2(lowerleg_length * sin(knee_angle), upperleg_length + lowerleg_length * cos(knee_angle)));

        result(0,i) = hip_rolling_angle;
        result(1,i) = hip_pitching_angle;
        result(2,i) = knee_angle;
    }
    return result;
}

void robot::Testudog::push_up(double speed){
    double t = double(count)/60;

    Eigen::Matrix<double,3,4> pos;
    //FL
    pos(0,0) = x_global + body_width/2 + hip_length;
    pos(1,0) = y_global - body_front;
    pos(2,0) = 0;
    //FR
    pos(0,1) = x_global - body_width/2 - hip_length;
    pos(1,1) = y_global - body_front;
    pos(2,1) = 0;
    //BL
    pos(0,2) = x_global + body_width/2 + hip_length;
    pos(1,2) = y_global + body_back;
    pos(2,2) = 0;
    //BR
    pos(0,3) = x_global - body_width/2 - hip_length;
    pos(1,3) = y_global + body_back;
    pos(2,3) = 0;
    
    z_global = 0.16 + 0.04*sin(speed*t);

    legpos_global = pos;
    joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
    count += 1;
}

void robot::Testudog::twist_yaw(double speed){
    
    double t = double(count)/60;
    yaw = 0.4*sin(speed*t);
    z_global = 0.15;
    double x_offset = 0.0;
    double y_offset = 0.0;

    Eigen::Matrix<double,3,4> pos;
    //FL
    pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,0) = y_global - body_front + y_offset;
    pos(2,0) = 0;
    //FR
    pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,1) = y_global - body_front + y_offset;
    pos(2,1) = 0;
    //BL
    pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,2) = y_global + body_back + y_offset;
    pos(2,2) = 0;
    //BR
    pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,3) = y_global + body_back + y_offset;
    pos(2,3) = 0;

    legpos_global = pos;
    joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
    count += 1;
}

void robot::Testudog::twist_roll(double speed){

    double t = double(count)/60;
    roll = 0.23*sin(speed*t);
    z_global = 0.15-0.02*sin(speed*t);
    double x_offset = 0.0;
    double y_offset = 0.0;

    Eigen::Matrix<double,3,4> pos;
    //FL
    pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,0) = y_global - body_front + y_offset;
    pos(2,0) = 0;
    //FR
    pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,1) = y_global - body_front + y_offset;
    pos(2,1) = 0;
    //BL
    pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,2) = y_global + body_back + y_offset;
    pos(2,2) = 0;
    //BR
    pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,3) = y_global + body_back + y_offset;
    pos(2,3) = 0;

    legpos_global = pos;
    joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
    count += 1;
}

void robot::Testudog::twist_pitch(double speed){
    
    double t = double(count)/60;
    pitch = 0.3*sin(speed*t);
    z_global = 0.17;
    double x_offset = 0.0;
    double y_offset = 0.0;

    Eigen::Matrix<double,3,4> pos;
    //FL
    pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,0) = y_global - body_front + y_offset;
    pos(2,0) = 0;
    //FR
    pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,1) = y_global - body_front + y_offset;
    pos(2,1) = 0;
    //BL
    pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,2) = y_global + body_back + y_offset;
    pos(2,2) = 0;
    //BR
    pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,3) = y_global + body_back + y_offset;
    pos(2,3) = 0;

    legpos_global = pos;
    joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
    count += 1;
}

void robot::Testudog::trot(double step_size, double step_height, double step_time){
    z_global = 0.18;
    double t = double(count)/60;
    double x_offset;
    double y_offset;
    double proportion = t/step_time;
    double dz_foot;
    double dy_foot;
    double body_sideway_shift = 0.008; // effort controller
    // double body_sideway_shift = 0.019; // position controller
    // roll = robot::Testudog::orientation_control()(0);
    // pitch = robot::Testudog::orientation_control()(1);

    if(proportion <= 1.0){
        dy_foot = proportion*step_size - step_size/2;
        x_offset = -body_sideway_shift*cos(proportion*M_PI);
        y_offset = proportion*step_size/2 - step_size/4;

        if(proportion < 0.5){
            dz_foot = 2*proportion*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*(proportion - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset - dy_foot;
        pos(2,0) = dz_foot;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset;
        pos(2,1) = 0;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset;
        pos(2,2) = 0;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset - dy_foot;
        pos(2,3) = dz_foot;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else if(proportion <= 2.0 && proportion > 1.0){
        dy_foot = (proportion-1)*step_size - step_size/2;
        x_offset = body_sideway_shift*cos((proportion-1)*M_PI);
        y_offset = (proportion-1)*step_size/2 - step_size/4;

        if((proportion-1) < 0.5){
            dz_foot = 2*(proportion-1)*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*((proportion-1) - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset;
        pos(2,0) = 0;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset - dy_foot;
        pos(2,1) = dz_foot;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset - dy_foot;
        pos(2,2) = dz_foot;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset;
        pos(2,3) = 0;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;
    }else{
        count = 0;
    }
}

void robot::Testudog::trot_planning(double step_size, double step_rot,  double step_height, double step_time){
    z_global = 0.18;
    double t = double(count)/60;
    double x_offset;
    double x_offset_rot;
    double y_offset;
    double proportion = t/step_time;
    double dz_foot;
    double dy_foot;
    double dx_foot;
    double body_sideway_shift = 0.008; // effort controller

    if(proportion <= 1.0){
        dy_foot = proportion*step_size - step_size/2;
        dx_foot = proportion*step_rot - step_rot/2;
        x_offset = -body_sideway_shift*cos(proportion*M_PI);
        x_offset_rot = step_rot/4 - proportion*step_rot/2;
        y_offset = proportion*step_size/2 - step_size/4;

        if(proportion < 0.5){
            dz_foot = 2*proportion*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*(proportion - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset + x_offset_rot + dx_foot;
        pos(1,0) = y_global - body_front + y_offset - dy_foot;
        pos(2,0) = dz_foot;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset + x_offset_rot;
        pos(1,1) = y_global - body_front + y_offset;
        pos(2,1) = 0;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset - x_offset_rot;
        pos(1,2) = y_global + body_back + y_offset;
        pos(2,2) = 0;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset - x_offset_rot - dx_foot;
        pos(1,3) = y_global + body_back + y_offset - dy_foot;
        pos(2,3) = dz_foot;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else if(proportion <= 2.0 && proportion > 1.0){
        dy_foot = (proportion-1)*step_size - step_size/2;
        dx_foot = (proportion-1)*step_rot - step_rot/2;
        x_offset = body_sideway_shift*cos((proportion-1)*M_PI);
        x_offset_rot = step_rot/4 - (proportion-1)*step_rot/2;
        y_offset = (proportion-1)*step_size/2 - step_size/4;

        if((proportion-1) < 0.5){
            dz_foot = 2*(proportion-1)*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*((proportion-1) - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset + x_offset_rot;
        pos(1,0) = y_global - body_front + y_offset;
        pos(2,0) = 0;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset + x_offset_rot + dx_foot;
        pos(1,1) = y_global - body_front + y_offset - dy_foot;
        pos(2,1) = dz_foot;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset - x_offset_rot - dx_foot;
        pos(1,2) = y_global + body_back + y_offset - dy_foot;
        pos(2,2) = dz_foot;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset - x_offset_rot;
        pos(1,3) = y_global + body_back + y_offset;
        pos(2,3) = 0;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;
    }else{
        count = 0;
    }
}

void robot::Testudog::crawl(double step_size, double step_height, double step_time){

    double t = double(count)/60;
    double x_offset;
    double y_offset;
    double proportion = t/step_time;
    double dz_foot;
    double dy_foot;
    double body_sideway_shift = 0.032; // to maintain CM btw 2 foot best for pos controller 0.019
    z_global = 0.18;

    if(proportion <= 1.0){ //BL
        dy_foot = proportion*step_size - step_size;
        x_offset = body_sideway_shift;
        y_offset = proportion*step_size/4 - 4*step_size/8;

        if(proportion < 0.5){
            dz_foot = 2*proportion*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*(proportion - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset + 3*step_size/4;
        pos(2,0) = 0;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset + step_size/4;
        pos(2,1) = 0;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset - dy_foot;
        pos(2,2) = dz_foot;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset + 2*step_size/4;
        pos(2,3) = 0;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else if(proportion <= 2.0 && proportion > 1.0){ //FL
        dy_foot = (proportion-1)*step_size - step_size;
        x_offset = body_sideway_shift;
        y_offset = (proportion-1)*step_size/4 - 4*step_size/8;

        if((proportion-1) < 0.5){
            dz_foot = 2*(proportion-1)*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*((proportion-1) - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset - dy_foot;
        pos(2,0) = dz_foot;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset + 2*step_size/4;
        pos(2,1) = 0;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset + step_size/4;
        pos(2,2) = 0;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset + 3*step_size/4;
        pos(2,3) = 0;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else if(proportion <= 2.5 && proportion > 2.0){ //body shift
        x_offset = body_sideway_shift*(0.25-(proportion-2))*4;
        y_offset = step_size/4 - 4*step_size/8;

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset;
        pos(2,0) = 0;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset + 2*step_size/4;
        pos(2,1) = 0;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset + step_size/4;
        pos(2,2) = 0;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset + 3*step_size/4;
        pos(2,3) = 0;     

        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else if(proportion <= 3.5 && proportion > 2.5){ //BR
        dy_foot = (proportion-2.5)*step_size - step_size;
        x_offset = -body_sideway_shift;
        y_offset = (proportion-2.5)*step_size/4 - 4*step_size/8;

        if((proportion-2.5) < 0.5){
            dz_foot = 2*(proportion-2.5)*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*((proportion-2.5) - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset + step_size/4;
        pos(2,0) = 0;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset + 3*step_size/4;
        pos(2,1) = 0;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset + 2*step_size/4;
        pos(2,2) = 0;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset - dy_foot;
        pos(2,3) = dz_foot;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else if(proportion <= 4.5 && proportion > 3.5){ //FR
        dy_foot = (proportion-3.5)*step_size - step_size;
        x_offset = -body_sideway_shift;
        y_offset = (proportion-3.5)*step_size/4 - 4*step_size/8;

        if((proportion-3.5) < 0.5){
            dz_foot = 2*(proportion-3.5)*step_height;
        } else{
            dz_foot = step_height*(1.0 - 2.0*((proportion-3.5) - 0.5)); 
        }

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset + 2*step_size/4;
        pos(2,0) = 0;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset - dy_foot;
        pos(2,1) = dz_foot;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset + 3*step_size/4;
        pos(2,2) = 0;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset + step_size/4;
        pos(2,3) = 0;
        
        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else if(proportion <= 5.0 && proportion > 4.5){ //body shift
        x_offset = -body_sideway_shift*(0.25-(proportion-4.5))*4;
        y_offset = step_size/4 - 4*step_size/8;

        Eigen::Matrix<double,3,4> pos;
        //FL
        pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,0) = y_global - body_front + y_offset + 2*step_size/4;
        pos(2,0) = 0;
        //FR
        pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,1) = y_global - body_front + y_offset;
        pos(2,1) = 0;
        //BL
        pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
        pos(1,2) = y_global + body_back + y_offset + 3*step_size/4;
        pos(2,2) = 0;
        //BR
        pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
        pos(1,3) = y_global + body_back + y_offset + step_size/4;
        pos(2,3) = 0;  

        legpos_global = pos;
        joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
        count += 1;

    }else{
        count = 0;
    }
}

void robot::Testudog::get_imu(const sensor_msgs::Imu::ConstPtr& msg){
    double x = msg->orientation.x;
    double y = msg->orientation.y;
    double z = msg->orientation.z;
    double w = msg->orientation.w;

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    imu_roll = roll;
    imu_pitch = pitch;
}

void robot::Testudog::get_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg){  
    vel_cmd = msg->linear.x;
    ang_cmd = msg->angular.z;
    vel_limit();
}

void robot::Testudog::vel_limit(){
    double lin_vel_max = 0.4;
    double ang_vel_max = 0.2;
    if(vel_cmd > lin_vel_max){
        lin_vel = lin_vel_max;
    }else if(vel_cmd < -lin_vel_max){
        lin_vel = -lin_vel_max;
    }else{
        lin_vel = vel_cmd;
    }
    if(ang_cmd > ang_vel_max){
        ang_vel = ang_vel_max;
    }else if(ang_cmd < -ang_vel_max){
        ang_vel = -ang_vel_max;
    }else{
        ang_vel = ang_cmd;
    }
}

int robot::Testudog::getIndex(std::vector<std::string> v, std::string value){
    for(int i = 0; i < v.size(); i++)
    {
        if(v[i].compare(value) == 0)
            return i;
    }
    return -1;
}

void robot::Testudog::get_traj(const geometry_msgs::Point::ConstPtr& msg){
    x_goal = msg->x;
    y_goal = msg->y;
}

void robot::Testudog::get_state(const gazebo_msgs::ModelStates::ConstPtr& msg){
    int idx = getIndex(msg->name, "robot_model");
    x_pos = msg->pose[idx].position.x;
    y_pos = msg->pose[idx].position.y;
    double orien_w = msg->pose[idx].orientation.w;
    double orien_x = msg->pose[idx].orientation.x;
    double orien_y = msg->pose[idx].orientation.y;
    double orien_z = msg->pose[idx].orientation.z;

    tf::Quaternion q(orien_x, orien_y, orien_z, orien_w);
    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_pos = yaw;
}

void robot::Testudog::reach_goal(){
    double kp_dist = 1;
    double kp_ang = 0.5;
    double dist = sqrt(pow(x_pos-x_goal, 2) + pow(y_pos-y_goal, 2));
    double ang = atan2(y_pos-y_goal, x_pos-x_goal);
    double ang_offset = ang-M_PI/2 + M_PI;
    double yaw_offset = yaw_pos + M_PI;

    std::cout<< x_pos << std::endl;
    std::cout<< y_pos << std::endl;
    std::cout<< " " << std::endl;

    // normalize angle_offset
    while (ang_offset > M_PI){
        ang_offset = ang_offset - 2*M_PI;
    }
    while (ang_offset < -M_PI){
        ang_offset = ang_offset + 2*M_PI;
    }
    // normalize yaw_offset
    while (yaw_offset > M_PI){
        yaw_offset = yaw_offset - 2*M_PI;
    }
    while (yaw_offset < -M_PI){
        yaw_offset = yaw_offset + 2*M_PI;
    }


    vel_cmd = kp_dist*dist;
    ang_cmd = kp_ang*(ang_offset-yaw_offset);

    // std::cout<< ang_offset << std::endl;
    // std::cout<< yaw_offset << std::endl;
    // std::cout<< " " << std::endl;
    vel_limit();
}

float robot::Testudog::get_x_error(){
    return x_goal-x_pos;
}

float robot::Testudog::get_y_error(){
    return y_goal-y_pos;
}

Eigen::Array2d robot::Testudog::orientation_control(){
    double kp = 0.5;
    double kd = 0;
    double ki = 0.6;

    std_msgs::String msg;
	std::stringstream ss;

    Eigen::Array2d error;
    Eigen::Array2d derror;
    static Eigen::Array2d ierror{Eigen::Array2d::Zero(2)};
    static Eigen::Array2d prev_error{Eigen::Array2d::Zero(2)};
    static double t0{0};
    double i_max = 0.5;

    ros::Time t = ros::Time::now();
    double dt = t.toSec() - t0;

    error(0) = 0 - imu_roll;
    error(1) = 0 - imu_pitch;

    if(dt == 0){
        derror = Eigen::Array2d::Zero(2);
    }else{
        derror = (error - prev_error)/dt;
    }

    ierror += error*dt;

    for(int i = 0; i <= 1; i++){
    if(ierror(i) < -i_max)
        ierror(i) = -i_max;
    else if(ierror(i) > i_max)
        ierror(i) = i_max;
    }

    Eigen::Array2d result = kp*error + kd*derror + ki*ierror;

    ss << imu_roll << '\n';
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    prev_error = error;
    t0 = t.toSec();

    return result;
}

void robot::Testudog::balance(){
    double t = double(count)/60;
    roll = robot::Testudog::orientation_control()(0);
    pitch = robot::Testudog::orientation_control()(1);
    z_global = 0.16;
    double x_offset = 0.0;
    double y_offset = 0.0;

    Eigen::Matrix<double,3,4> pos;
    //FL
    pos(0,0) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,0) = y_global - body_front + y_offset;
    pos(2,0) = 0;
    //FR
    pos(0,1) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,1) = y_global - body_front + y_offset;
    pos(2,1) = 0;
    //BL
    pos(0,2) = x_global + body_width/2 + hip_length + x_offset;
    pos(1,2) = y_global + body_back + y_offset;
    pos(2,2) = 0;
    //BR
    pos(0,3) = x_global - body_width/2 - hip_length + x_offset;
    pos(1,3) = y_global + body_back + y_offset;
    pos(2,3) = 0;

    legpos_global = pos;
    joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
    count += 1;
}

void robot::Testudog::circle(double speed){
    double t = double(count)/60;

    Eigen::Matrix<double,3,4> pos;
    //FL
    pos(0,0) = x_global + body_width/2 + hip_length + 0.042*cos(speed*t);
    pos(1,0) = y_global - body_front + 0.042*sin(speed*t);
    pos(2,0) = 0.042*sin(speed*t);
    //FR
    pos(0,1) = x_global - body_width/2 - hip_length + 0.042*cos(speed*t);
    pos(1,1) = y_global - body_front + 0.042*sin(speed*t);
    pos(2,1) = 0.042*sin(speed*t);
    //BL
    pos(0,2) = x_global + body_width/2 + hip_length + 0.042*cos(speed*t);
    pos(1,2) = y_global + body_back + 0.042*sin(speed*t);
    pos(2,2) = 0.042*sin(speed*t);
    //BR
    pos(0,3) = x_global - body_width/2 - hip_length + 0.042*cos(speed*t);
    pos(1,3) = y_global + body_back + 0.042*sin(speed*t);
    pos(2,3) = 0.042*sin(speed*t);
    
    z_global = 0.16;

    legpos_global = pos;
    joint_angle = inv_kine(global2local_legpos(legpos_global,x_global,y_global,z_global,roll,pitch,yaw));
    count += 1;
}