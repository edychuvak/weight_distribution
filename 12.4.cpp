void physics_trailer_u::assign_weight_distribution(const float weight_on_rear_hook, const float weight_on_front_hook)
{

	// Weight distribution on trailer axles consists two components of mass. One is mass of trailer applied on base center of trailer mass.
	// The second component is mass carried from slave trailer hook. There are two equations to solve for each component:
	// 1) the mass conservation or second Newton's law for longitudinal direction:
	//					m * g = sum (weight_distribution)
	//					carried_mass * g = sum (carried_weight_distribution)
	// 2) the moment equation
	//					m * g * center_of_mass = sum (weight_distribution * ith_axle_position)
	//					carried_mass * g * hook_pos = sum (carried_weight_distribution * ith_axle_position)
	// The algorithm sets the same (both base and carried) weight distribution on all axles and calculates carried weight distribution on the hook.
	// (on the axles)
	//				base_weight_distribution = m * g ( hook_position - cog_position) / (n * hook_posision - sum (axle_position)
	//				carried_weight_distribution = carried_mass * g ( hook_position - rear_hook_position) / ( n * hook_position - sum (axle_position))
	// (on the hook)
	//				base_weight_distribution  = m * g - sum (weight_distribution)
	//				carried_weight_distribution = carried_mass - sum (carried_weight_distribution)

	const unsigned total_axle_count = chassis_data->get_wheel_count() / 2u;
	v_assert(weight_distribution.get_size() == total_axle_count);
	const float total_weight = vehicle->get_mass();
	float axles_moment = 0.0f;

	// Variable axle_on_ground determine the approximate portion of axles on ground (as a float value). For example, if
	// one axle is 60% lifted, it contributes 0.4f to the axle_on_ground variable. Used to calculate weight_distribution w.r.t lifted axles.

	float axle_on_ground = 0.0f;
	for (size_t idx = 0; idx < total_axle_count; ++idx) {
		axles_moment += (chassis_data->get_init_wheel_position(2 * idx)[2]) * transform_lift_ratio(idx);
		message("axle mom %f", axles_moment);
		axle_on_ground += transform_lift_ratio(idx);
	}
	const float local_hook_position = get_local_hook_position()[2];
	float weight_on_axle = total_weight * (local_hook_position - chassis_actor_offset[2]) / (axle_on_ground * local_hook_position - axles_moment);
	if (weight_on_rear_hook > 0.0f) {
		const float slave_hook_pos = get_slave_hook_position()[2];
		weight_on_axle += weight_on_rear_hook * (local_hook_position - slave_hook_pos) / (axle_on_ground * local_hook_position - axles_moment);
	}
	else if (weight_on_front_hook > 0.0f) {
		const float slave_hook_pos = get_slave_hook_position()[2];
		weight_on_axle += weight_on_front_hook * (local_hook_position - slave_hook_pos) / (axle_on_ground * local_hook_position - axles_moment);
	}

	weight_on_hook = total_weight + weight_on_rear_hook + weight_on_front_hook;
	float sum =0.0f;
	float summ = 0.0f;
	for (size_t idx = 0; idx < total_axle_count; ++idx) {
		const float transformed_weight_on_axle = transform_lift_ratio(idx) * weight_on_axle;
		sum += transformed_weight_on_axle * (chassis_data->get_init_wheel_position(2 * idx)[2]);
		summ += transformed_weight_on_axle;
		weight_distribution[idx] = transformed_weight_on_axle * 0.5f * EARTH_GRAVITY;
		weight_on_hook -= transformed_weight_on_axle;
	}
	message(" lhp %f", local_hook_position);
	message("suma %f",chassis_actor_offset[2]);
	message("suma %f", (sum + local_hook_position*weight_on_hook) / (summ + weight_on_hook));
	message("suma %f", axles_moment * weight_on_axle / (total_weight + weight_on_rear_hook + weight_on_front_hook - weight_on_hook ));
	message("suma %f", axles_moment /total_axle_count);
	//chassis_actor_offset[2] = axles_moment / total_axle_count;
}

void physics_vehicle_u::assign_weight_distribution(const float weight_on_rear_hook, const float weight_on_front_hook)
{
	UNREFERENCED(weight_on_front_hook);

	// Weight distribution on truck axles consists two components of mass. One is base weight distribution on axle,
	// taken from chassis data. The second component is mass carried from trailer hook. There are two equations to solve:
	// 1) the mass conservation or second Newton's law for longitudinal direction:
	//					m * g = sum (weight_distribution)
    // 2) the moment equation
	//					m * g * hook_pos = sum (weight_distribution * ith_axle_position)
    // In case of only two axles, only one solution exists. For more than 2 axles all possible weight distributions
	// are linearly dependent on each other. We choose all axles except the first to have equal carried weight distribution
	// and subsequently calculate the carried weight on the first axle.
    // i=!1; weight_distribution =  m * g (first_axle_position - hook_position)/ ( (n-1)* hook_position - sum (2th_axle_position...nth_axle_position) )
	// i=1; weight_distribution = m * g - sum ( weight_distribution )

	const unsigned total_axle_count = chassis_data->get_wheel_count() / 2u;
	v_assert(weight_distribution.get_size() == total_axle_count);
	const size_t kerb_weight_size = chassis_data->kerb_weight.get_size();
	const float total_weight = vehicle->get_mass();
	float axles_moment = 0.0f;
	weight_distribution[0] = chassis_data->kerb_weight[0];
	for (size_t idx = 1; idx < total_axle_count; ++idx) {
		axles_moment += chassis_data->get_init_wheel_position(2 * idx)[2];
		weight_distribution[idx] = chassis_data->kerb_weight[idx];
	}
	float carried_weight_first_axle = weight_on_rear_hook;
	const float slave_hook_pos = chassis_data->hook_position[2];
	const float first_wheel_z_pos = chassis_data->get_init_wheel_position(2 * 0)[2];
	const float carried_weight_idx = weight_on_rear_hook * (first_wheel_z_pos - slave_hook_pos) / ((total_axle_count - 1) * first_wheel_z_pos - axles_moment);
	for (size_t idx = 1; idx < total_axle_count; ++idx) {
		weight_distribution[idx] += carried_weight_idx;
		carried_weight_first_axle -= carried_weight_idx;
		const float wheel_z_pos = chassis_data->get_init_wheel_position(2 * idx)[2];
		if (transform_lift_ratio(idx) < 1.0f) {
			const float lifted_axle_moment = axles_moment - chassis_data->get_init_wheel_position(2 * idx)[2];
			float lifted_weight_first_axle = weight_distribution[idx] * (1.0f - transform_lift_ratio(idx));
			const float lifted_weight_idx = lifted_weight_first_axle * (first_wheel_z_pos - wheel_z_pos) / ((total_axle_count - 2) * first_wheel_z_pos - lifted_axle_moment);
			for (size_t idx_l = 1; idx_l < total_axle_count; ++idx_l) {
				if (idx_l != idx) {
					weight_distribution[idx_l] += lifted_weight_idx;
					lifted_weight_first_axle -= lifted_weight_idx;
				}
			}
			weight_distribution[0] += lifted_weight_first_axle;
			weight_distribution[idx] *= transform_lift_ratio(idx);
		}
	}
	weight_distribution[0] += carried_weight_first_axle;
	message("zero truck %f", chassis_actor_offset[2]);

	for (size_t idx = 0; idx < total_axle_count; ++idx) {
		weight_distribution[idx] *= 0.5f * EARTH_GRAVITY;
	}
}