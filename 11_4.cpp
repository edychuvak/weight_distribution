float vehicle_shared_u::calculate_z_cog(void)
{
	const unsigned axles_count = chassis_data->get_wheel_count() / 2;
	unsigned front_axle_count = 0;
	unsigned rear_axle_count = 0;
	float cog_shift = 0.0f;
	float front_z_cog = 0.0f;
	float rear_z_cog = 0.0f;
	weight_on_hook = 0.0f;

	// Determine the front/rear axles based on the model center.
	for (unsigned index = 0; index < axles_count; index++) {
		const float axle_z_pos = chassis_data->get_init_wheel_position(2 * index)[2];
		if (axle_z_pos < chassis_actor_offset[2]) {
			set_bit(front_axles, index);
			front_z_cog += axle_z_pos;
			front_axle_count++;
		}
		else {
			set_bit(rear_axles, index);
			rear_z_cog += axle_z_pos;
			rear_axle_count++;
		}
		axles_ordered[index] = index;
	}

	sort_trait_z_cog_t::chassis_data = chassis_data;
	axles_ordered.sort_by<sort_trait_z_cog_t>();

	if (chassis_data->weight_distribution == WEIGHT_DISTR_semitrailer || chassis_data->weight_distribution == WEIGHT_DISTR_perimeter) {

		// This actually produces precise results even when front
		// front wheels are present and the hook position would not have
		// to be taken into account.
		const float hook_z_position = chassis_data->hook_position[2];
		front_z_cog = (front_z_cog + hook_z_position) / (front_axle_count + 1);

		if (rear_axle_count == 0) {
			const float slave_hook_z_position = static_cast<physics_trailer_u*>(this)->get_slave_hook_position()[2];
			rear_z_cog = (rear_z_cog + slave_hook_z_position) / (rear_axle_count + 1);
		}
		else {
			rear_z_cog = rear_z_cog / rear_axle_count;
		}

		const float front_weight_rate = front_axle_count == 0 ? (rear_z_cog - chassis_actor_offset[2]) / (rear_z_cog - front_z_cog) : static_cast<float>(front_axle_count + 2) / static_cast<float>(axles_count + 2);
		const float vehicle_mass = vehicle->get_mass();
		const float weight_front = vehicle_mass * front_weight_rate;
		// We need minimal weight on hook for traction, see comment at HOOK_WEIGHT_BOOST definition.
		weight_on_hook = v_max(weight_front / (front_axle_count + 1), HOOK_WEIGHT_BOOST * vehicle_mass);
		base_weight_on_hook = weight_on_hook;
		cog_shift = front_axle_count == 0 ? 0.0f : (rear_z_cog + front_z_cog) / 2.0f;

		// For the perimeter frame trailer, we have to transfer some weight back to the slave.
		// Currently, we are using this only for an ATS oversize mission, where the heavy weight of the cargo
		// calls for higher update frequency of the supporting dolly. Should be revisited once we add more
		// trailers of this type.
		if (chassis_data->weight_distribution == WEIGHT_DISTR_perimeter) {
			v_assert(static_cast<physics_trailer_u*>(this)->get_slave_trailer());
			const float weight_to_slave = vehicle_mass - weight_front;
			static_cast<physics_trailer_u*>(this)->get_slave_trailer()->recalculate_weight_distribution(weight_to_slave);
		}
	}
	else {
		front_z_cog = (front_axle_count == 0 ? 0.0f : front_z_cog / front_axle_count);
		rear_z_cog = (rear_axle_count == 0 ? 0.0f: rear_z_cog / rear_axle_count);

		if (chassis_data->kerb_weight.get_size() > 0) {
			float kerb_weight_sum = 0.0f;
			float kerb_weight_moment = 0.0f;

			for (size_t i = 0; i < axles_count; ++i) {
				kerb_weight_sum += chassis_data->kerb_weight[i];
				kerb_weight_moment += chassis_data->kerb_weight[i] * chassis_data->get_init_wheel_position(i * 2)[2];
			}
			cog_shift = kerb_weight_moment / kerb_weight_sum;
		}
		else {
			if (front_axle_count == 0) {
				cog_shift = rear_z_cog;
			}
			else if (rear_axle_count == 0) {
				cog_shift = front_z_cog;
			}
			else {
				cog_shift = (rear_z_cog + front_z_cog) / 2.0f;
			}
		}
	}
		return cog_shift;
}