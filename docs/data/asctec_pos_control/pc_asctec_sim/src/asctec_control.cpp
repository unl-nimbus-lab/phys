#include <asctec_control.h>

AscTec_Controller::AscTec_Controller(string qframe, string wframe, struct K_DATA * kvals)
{	
	q_frame = qframe;
	w_frame = wframe;

	battery = BATTERY_FULL;
	setParams(kvals);
	initAsctec();
}

AscTec_Controller::~AscTec_Controller() {}

struct PUB_DATA * AscTec_Controller::runAsctec(struct PUB_DATA * pub, struct GOAL_DATA * goal_in, tf::StampedTransform * transform)
{
	/* ---- Update Params and Status ---- */
	st.battery = pub->battery;
	AscTec_Controller::checkBattery();
	pub->g_feedback = *checkGoal(&pub->g_feedback);
	c.k_val = pub->k_val;

	/* ---- Update State ---- */
	AscTec_Controller::updatePosition(transform);
	if(pub->xyFree == 2) {
		st.vx = pub->vx;
		st.vy = pub->vy;
		st.ax = goal_in->goal.ax;
		st.ay = goal_in->goal.ay;
	
		c.k_val.kpx *= 0.2; //7.5
		c.k_val.kpy *= 0.2;   //6
	
		c.k_val.kix = 0;
		c.k_val.kiy = 0;

		c.k_val.kvx *= 1.28;   //7
		c.k_val.kvy *= 1.23;   //9

		c.k_val.kax = 0;
		c.k_val.kay = 0;
	}
	pub->state = *fillState(&pub->state);

	/* ---- Update Current Goal ---- */
	if(pub->running) {
		if(pub->xyFree == 1) {
			goal_in->isNew = true;
			updateGoalZY(goal_in);
			goal_in->isNew = false;

		}else{
			updateGoal(goal_in);
		}
	}else {
		freeGoal();
	}

	/* ---- Update Controller ---- */
	updateController();

	/* ---- Set TRPY Command ---- */
	if(pub->running) {
		pub->TRPYcmd = *setCmd(&pub->TRPYcmd);
	}else {
		pub->TRPYcmd.cmd[0] = true;
		pub->TRPYcmd.cmd[1] = true;
		pub->TRPYcmd.cmd[2] = true;
		pub->TRPYcmd.cmd[3] = true;
		pub->TRPYcmd.thrust = 0;
	}
	return pub;
}

void AscTec_Controller::setParams(struct K_DATA * k_ptr)
{
	c.k_val = *k_ptr;
}

pc_asctec_sim::pc_goal_cmd * AscTec_Controller::getGoal(void)
{
	pc_asctec_sim::pc_goal_cmd *ptr = new pc_asctec_sim::pc_goal_cmd;
	ptr->x = st.g_x;
	ptr->y = st.g_y;
	ptr->z = st.g_z;
	ptr->yaw = st.g_yaw;

	ptr->vx = st.g_vx;
	ptr->vy = st.g_vy;
	ptr->vz = st.g_vz;
	ptr->vyaw = st.g_vyaw;

	ptr->ax = st.g_ax;
	ptr->ay = st.g_ay;
	ptr->az = st.g_az;
	ptr->ayaw = st.g_ayaw;	

	ptr->goal_id = st.g_id;
	ptr->goal_limit = st.g_range;
	ptr->wait_time = st.wait_time;

	return ptr;
}

void AscTec_Controller::updatePosition(tf::StampedTransform * transform)
{
	st.dt = transform->stamp_.toSec() - st.past.toSec();

	//Set t-1 values
	st.x_p = st.x;
	st.y_p = st.y;
	st.z_p = st.z;
	st.yaw_p = st.yaw;
	st.vx_p = st.vx;
	st.vy_p = st.vy;
	st.vz_p = st.vz;
	st.vyaw_p = st.vyaw;
	st.past = transform->stamp_;

	//Get new position values
	st.yaw = tf::getYaw(transform->getRotation()) + 2*M_PI*st.yaw_counter;
	st.x = transform->getOrigin().x();
	st.y = transform->getOrigin().y();
	st.z = transform->getOrigin().z();

	//Adjust yaw counter
	float yaw_dif = st.yaw - st.yaw_p;
	if(abs(yaw_dif) > M_PI) {
		if(!st.yaw_check) {
			if(yaw_dif < 0.0) {
		 		st.yaw_counter += 1;
	 		}else {
		 		st.yaw_counter -= 1;
			}
			st.yaw_check = true;
		}else {
			st.yaw_check = false;
		}
	}

	//Calculate velocity values
	st.vx = ((st.x) - (st.x_p)) / st.dt;
	st.vy = ((st.y) - (st.y_p)) / st.dt;
	st.vz = ((st.z) - (st.z_p)) / st.dt;
	st.vyaw = ((st.yaw) - (st.yaw_p)) / st.dt;

	//Calculate accel values
	st.ax_buf[st.ax_n] = ((st.vx) - (st.vx_p)) / st.dt;
	st.ay_buf[st.ay_n] = ((st.vy) - (st.vy_p)) / st.dt;
	st.az_buf[st.az_n] = ((st.vz) - (st.vz_p)) / st.dt;
	st.ayaw_buf[st.ayaw_n] = ((st.vyaw) - (st.vyaw_p)) / st.dt;

	st.ax_n++;
	st.ay_n++;
	st.az_n++;
	st.ayaw_n++;

	st.ax_n &= BUFFER-1;
	st.ay_n &= BUFFER-1;
	st.az_n &= BUFFER-1;
	st.ayaw_n &= BUFFER-1;

	float tempx, tempy, tempz, tempyaw;
	tempx = tempy = tempz = tempyaw = 0.0;

	for(int i = 0; i < BUFFER; i++) {
		tempx += st.ax_buf[i];
		tempy += st.ay_buf[i];
		tempz += st.az_buf[i];
		tempyaw += st.ayaw_buf[i];
	}

	st.ax = tempx / BUFFER;
	st.ay = tempy / BUFFER;
	st.az = tempz / BUFFER;
	st.ayaw = tempyaw / BUFFER;
}

void AscTec_Controller::freeGoal()
{
	st.g_x = st.x;
	st.g_y = st.y;
	st.g_z = st.z;
	st.g_yaw = st.yaw;

	st.g_vx = 0;
	st.g_vy = 0;
	st.g_vz = 0;
	st.g_vyaw = 0;

	st.g_ax = 0;
	st.g_ay = 0;
	st.g_az = 0;
	st.g_ayaw = 0;

	st.g_range = 0;
	st.g_id = "init";
	st.wait_time = 0;
}

void AscTec_Controller::updateGoal(struct GOAL_DATA * g_ptr)
{
	if(g_ptr->isNew) {
		st.g_x = g_ptr->goal.x;
		st.g_y = g_ptr->goal.y;
		st.g_z = g_ptr->goal.z;
		st.g_yaw = g_ptr->goal.yaw;

		st.g_vx = g_ptr->goal.vx;
		st.g_vy = g_ptr->goal.vy;
		st.g_vz = g_ptr->goal.vz;
		st.g_vyaw = g_ptr->goal.vyaw;

		st.g_ax = g_ptr->goal.ax;
		st.g_ay = g_ptr->goal.ay;
		st.g_az = g_ptr->goal.az;
		st.g_ayaw = g_ptr->goal.ayaw;

		st.g_range = g_ptr->goal.goal_limit;
		st.g_id = g_ptr->goal.goal_id;
		st.wait_time = g_ptr->goal.wait_time;
	}
}

void AscTec_Controller::updateGoalZY(struct GOAL_DATA * g_ptr)
{
	if(g_ptr->isNew) {
		st.g_x = st.x;
		st.g_y = st.y;
		st.g_z = g_ptr->goal.z;
		st.g_yaw = g_ptr->goal.yaw;

		st.g_vx = 0.0;
		st.g_vy = 0.0;
		st.g_vz = g_ptr->goal.vz;
		st.g_vyaw = g_ptr->goal.vyaw;

		st.g_ax = 0.0;
		st.g_ay = 0.0;
		st.g_az = g_ptr->goal.az;
		st.g_ayaw = g_ptr->goal.ayaw;

		st.g_range = g_ptr->goal.goal_limit;
		st.g_id = g_ptr->goal.goal_id;
		st.wait_time = g_ptr->goal.wait_time;
	}
}

void AscTec_Controller::updateGoalXZY(struct GOAL_DATA * g_ptr)
{
	if(g_ptr->isNew) {
		st.g_x = g_ptr->goal.x;
		st.g_y = st.y;
		st.g_z = g_ptr->goal.z;
		st.g_yaw = g_ptr->goal.yaw;

		st.g_vx = g_ptr->goal.vx;
		st.g_vy = 0.0;
		st.g_vz = g_ptr->goal.vz;
		st.g_vyaw = g_ptr->goal.vyaw;

		st.g_ax = g_ptr->goal.ax;
		st.g_ay = 0.0;
		st.g_az = g_ptr->goal.az;
		st.g_ayaw = g_ptr->goal.ayaw;

		st.g_range = g_ptr->goal.goal_limit;
		st.g_id = g_ptr->goal.goal_id;
		st.wait_time = g_ptr->goal.wait_time;
	}
}

void AscTec_Controller::updateController()
{
	/* ---- P calculation ---- */
	c.e_x = (st.g_x) - (st.x);
	c.e_y = (st.g_y) - (st.y);
	c.e_z = (st.g_z) - (st.z);
	c.e_yaw = (st.g_yaw) - (st.yaw);

	/* ---- I calculation ---- */
	c.i_x += (c.e_x) * st.dt;
	c.i_y += (c.e_y) * st.dt;
	c.i_z += (c.e_z) * st.dt;
	c.i_yaw += (c.e_yaw) * st.dt;

	/* ---- Vel calculation ---- */
	c.e_vx = (st.g_vx) - (st.vx);
	c.e_vy = (st.g_vy) - (st.vy);
	c.e_vz = (st.g_vz) - (st.vz); 
	c.e_vyaw = (st.g_vyaw) - (st.vyaw);

	/* ---- Acc calculation ---- */
	c.e_ax  = (st.g_ax) - (st.ax);
	c.e_ay = (st.g_ay) - (st.ay);
	c.e_az = (st.g_az) - (st.az); 
	c.e_ayaw = (st.g_ayaw) - (st.ayaw);

	/* ---- Windup Prevention ---- */
	c.i_x = limitOutput(c.i_x, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
	c.i_y = limitOutput(c.i_y, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
	c.i_z = limitOutput(c.i_z, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
	c.i_yaw = limitOutput(c.i_yaw, INTEGRAL_LIMIT, -INTEGRAL_LIMIT);
}

pc_asctec_sim::SICmd * AscTec_Controller::setCmd(pc_asctec_sim::SICmd * TRPY)
{
	double roll;
	double pitch;
	double thrust;
	double yaw;

	TRPY->cmd[0] = true;
	TRPY->cmd[1] = true;
	TRPY->cmd[2] = true;
	TRPY->cmd[3] = true;
	
	yaw = -(c.k_val.kpyaw * (c.e_yaw) + 
		c.k_val.kiyaw * (c.i_yaw) + 
		c.k_val.kvyaw * (c.e_vyaw) +
		c.k_val.kayaw * (c.e_ayaw));

	yaw = limitOutput(yaw, YAW_MAX, YAW_MIN);
	
	if(!isnan(M_PI * (yaw / YAW_MAX))) {
		TRPY->yaw = M_PI * (yaw / YAW_MAX);
	}

	pitch = -((c.k_val.kpx * (c.e_x) + 
		c.k_val.kix * (c.i_x) + 
		c.k_val.kvx * (c.e_vx ) + 
		c.k_val.kax * (c.e_ax)) *
		cos(st.yaw)) + 

		-((c.k_val.kpy * (c.e_y) + 
		c.k_val.kiy * (c.i_y) + 
		c.k_val.kvy * (c.e_vy) + 
		c.k_val.kay * (c.e_ay)) *
		sin(st.yaw)); 

	pitch = limitOutput(pitch, PITCH_MAX, PITCH_MIN);

	if(!isnan(M_PI * (pitch / PITCH_MAX) / BOUNDED_ANGLE)) {
		TRPY->pitch = M_PI * (pitch / PITCH_MAX) / BOUNDED_ANGLE + PBIAS;
	}

	roll = ((c.k_val.kpx * (c.e_x) + 
		c.k_val.kix * (c.i_x) + 
		c.k_val.kvx * (c.e_vx) +
		c.k_val.kax * (c.e_ax)) *
		sin(st.yaw)) +

		-((c.k_val.kpy * (c.e_y) + 
		c.k_val.kiy * (c.i_y) + 
		c.k_val.kvy * (c.e_vy) + 
		c.k_val.kay * (c.e_ay)) *
		cos(st.yaw));

	roll = limitOutput(roll, ROLL_MAX, ROLL_MIN);

	if(!isnan(M_PI * (roll / ROLL_MAX) / BOUNDED_ANGLE)) {
		TRPY->roll = M_PI * (roll / ROLL_MAX) / BOUNDED_ANGLE + RBIAS;
	}

	thrust = c.k_val.kpz * (c.e_z) + 
		c.k_val.kiz * (c.i_z) + 
		c.k_val.kvz * (c.e_vz) +
		c.k_val.kaz * (c.e_az) + G_TH;

	thrust = limitOutput(thrust, THRUST_MAX, THRUST_MIN);

	if(!isnan(thrust / THRUST_MAX)) {
		TRPY->thrust = thrust / THRUST_MAX;
	}

	return TRPY;
}

pc_asctec_sim::pc_state * AscTec_Controller::fillState(pc_asctec_sim::pc_state * out_ptr)
{
	out_ptr->event = st.past;

	out_ptr->x = st.x;
	out_ptr->vx = st.vx;
	out_ptr->ax = st.ax;
	
	out_ptr->y = st.y;
	out_ptr->vy = st.vy;
	out_ptr->ay = st.ay;

	out_ptr->z = st.z;
	out_ptr->vz = st.vz;
	out_ptr->az = st.az;

	out_ptr->yaw = st.yaw;
	out_ptr->vyaw = st.vyaw;
	out_ptr->ayaw = st.ayaw;
	
	return out_ptr;
}

void AscTec_Controller::initAsctec()
{
	/* ----- Controller Init ----- */
	c.e_x = 0.0;
	c.i_x = 0.0;
	c.e_vx = 0.0;

	c.e_y = 0.0;
	c.i_y = 0.0;
	c.e_vy = 0.0;

	c.e_z = 0.0;
	c.i_z = 0.0;
	c.e_vz = 0.0;

	c.e_yaw = 0.0;
	c.i_yaw = 0.0;
	c.e_vyaw = 0.0;

	/* ----- Position Init ----- */
	st.x = 0.0;
	st.y = 0.0;
	st.z = 0.0;
	st.yaw = 0.0;

	st.x_p = 0.0;
	st.y_p = 0.0;
	st.z_p = 0.0;
	st.yaw_p = 0.0;

	st.vx = 0.0;
	st.vy = 0.0;
	st.vz = 0.0;
	st.vyaw = 0.0;

	st.g_x = 0.0;
	st.g_y = 0.0;
	st.g_z = 0.0;
	st.g_yaw = 0.0;

	st.g_vx = 0.0;
	st.g_vy = 0.0;
	st.g_vz = 0.0;
	st.g_vyaw = 0.0;

	st.ax_n = 0;
	st.ay_n = 0;
	st.az_n = 0;
	st.ayaw_n = 0;

	st.yaw_counter = 0;

	for(int i = 0; i < BUFFER; i++) {
		st.ax_buf[i] = 0;
		st.ay_buf[i] = 0;
		st.az_buf[i] = 0;
		st.ayaw_buf[i] = 0;
	}

	st.g_range = 0.05;
	st.wait_time = 0.0;
	st.waiting = false;
	st.g_arrival = false;
	st.g_id = "Init";
}

void AscTec_Controller::checkBattery()
{
	if(st.battery <= BATTERY_MID && st.battery_status == 3) {
		ROS_INFO("50 Percent Battery Remaining: %f V", st.battery);
		st.battery_status = 2;	

	}else if(st.battery <= BATTERY_EMPTY && st.battery_status == 2) {
		ROS_INFO("10 Percent Battery Remaining: %f V, land now!!", st.battery);
		st.battery_status = 1;
	}
}

pc_asctec_sim::pc_feedback * AscTec_Controller::checkGoal(pc_asctec_sim::pc_feedback * on_goal)
{
	float distance = sqrt((c.e_x) * (c.e_x) + 
			(c.e_y) * (c.e_y) + 
			(c.e_z) * (c.e_z)); 

	if((distance <= (st.g_range)) && not (st.g_arrival)) {
		st.wait_start = ros::Time::now().toSec(); 
		st.waiting = true;  
		st.g_arrival = true;
	}
	if(st.waiting) {
		if((ros::Time::now().toSec() - st.wait_start) >= st.wait_time) {
			st.waiting = false;
			on_goal->goal_id = st.g_id;
			on_goal->event = ros::Time::now();
			on_goal->arrived = true;
		}
	}
	return on_goal;
}

float AscTec_Controller::limitOutput(float input, float ceiling, float floor) 
{
	if(input > ceiling) {
		return ceiling;
	}else if(input < floor) {
		return floor;
	}else {
		return input;
	}
}
