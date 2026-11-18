#include "advanced_ai_navigator_3d.hpp"

#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/collision_object3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/navigation_server3d.hpp>
#include <godot_cpp/classes/physics_direct_space_state3d.hpp>
#include <godot_cpp/classes/physics_ray_query_parameters3d.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <cmath>

using namespace godot;

AdvancedAINavigator3D::AdvancedAINavigator3D() {
	query_params.instantiate();
	query_result.instantiate();
}

AdvancedAINavigator3D::~AdvancedAINavigator3D() {
	_free_server_agent();
}

void AdvancedAINavigator3D::_bind_methods() {
	// Enum
	BIND_ENUM_CONSTANT(STEERING_OMNI);
	BIND_ENUM_CONSTANT(STEERING_VEHICLE);

	// Control Methods
	ClassDB::bind_method(D_METHOD("set_target_position", "target"), &AdvancedAINavigator3D::set_target_position);
	ClassDB::bind_method(D_METHOD("get_target_position"), &AdvancedAINavigator3D::get_target_position);
	ClassDB::bind_method(D_METHOD("force_repath"), &AdvancedAINavigator3D::force_repath);
	ClassDB::bind_method(D_METHOD("stop_navigation"), &AdvancedAINavigator3D::stop_navigation);
	ClassDB::bind_method(D_METHOD("is_navigation_finished"), &AdvancedAINavigator3D::is_navigation_finished);

	// Inspection & Vehicle Hints
	ClassDB::bind_method(D_METHOD("get_current_path"), &AdvancedAINavigator3D::get_current_path);
	ClassDB::bind_method(D_METHOD("get_next_waypoint"), &AdvancedAINavigator3D::get_next_waypoint);
	ClassDB::bind_method(D_METHOD("get_desired_velocity"), &AdvancedAINavigator3D::get_desired_velocity);
	ClassDB::bind_method(D_METHOD("get_safe_velocity"), &AdvancedAINavigator3D::get_safe_velocity);
	ClassDB::bind_method(D_METHOD("get_steer_hint"), &AdvancedAINavigator3D::get_steer_hint);
	ClassDB::bind_method(D_METHOD("get_throttle_hint"), &AdvancedAINavigator3D::get_throttle_hint);
	ClassDB::bind_method(D_METHOD("get_remaining_distance"), &AdvancedAINavigator3D::get_remaining_distance);
	ClassDB::bind_method(D_METHOD("set_custom_navigation_map", "map"), &AdvancedAINavigator3D::set_custom_navigation_map);
	ClassDB::bind_method(D_METHOD("get_custom_navigation_map"), &AdvancedAINavigator3D::get_custom_navigation_map);

	// Properties
	ADD_GROUP("Navigation", "");
	ClassDB::bind_method(D_METHOD("set_steering_mode", "mode"), &AdvancedAINavigator3D::set_steering_mode);
	ClassDB::bind_method(D_METHOD("get_steering_mode"), &AdvancedAINavigator3D::get_steering_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "steering_mode", PROPERTY_HINT_ENUM, "Omni,Vehicle"), "set_steering_mode", "get_steering_mode");

	ClassDB::bind_method(D_METHOD("set_auto_move_parent_body", "enable"), &AdvancedAINavigator3D::set_auto_move_parent_body);
	ClassDB::bind_method(D_METHOD("is_auto_move_parent_body"), &AdvancedAINavigator3D::is_auto_move_parent_body);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_move_parent_body"), "set_auto_move_parent_body", "is_auto_move_parent_body");

	ClassDB::bind_method(D_METHOD("set_navigation_layers", "layers"), &AdvancedAINavigator3D::set_navigation_layers);
	ClassDB::bind_method(D_METHOD("get_navigation_layers"), &AdvancedAINavigator3D::get_navigation_layers);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "navigation_layers", PROPERTY_HINT_LAYERS_3D_NAVIGATION), "set_navigation_layers", "get_navigation_layers");

	ClassDB::bind_method(D_METHOD("set_waypoint_reached_distance", "distance"), &AdvancedAINavigator3D::set_waypoint_reached_distance);
	ClassDB::bind_method(D_METHOD("get_waypoint_reached_distance"), &AdvancedAINavigator3D::get_waypoint_reached_distance);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "waypoint_reached_distance", PROPERTY_HINT_RANGE, "0.1,10.0,0.05"), "set_waypoint_reached_distance", "get_waypoint_reached_distance");

	ClassDB::bind_method(D_METHOD("set_target_reached_distance", "distance"), &AdvancedAINavigator3D::set_target_reached_distance);
	ClassDB::bind_method(D_METHOD("get_target_reached_distance"), &AdvancedAINavigator3D::get_target_reached_distance);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "target_reached_distance", PROPERTY_HINT_RANGE, "0.1,15.0,0.05"), "set_target_reached_distance", "get_target_reached_distance");

	ClassDB::bind_method(D_METHOD("set_repath_interval", "seconds"), &AdvancedAINavigator3D::set_repath_interval);
	ClassDB::bind_method(D_METHOD("get_repath_interval"), &AdvancedAINavigator3D::get_repath_interval);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "repath_interval", PROPERTY_HINT_RANGE, "0.05,2.0,0.05"), "set_repath_interval", "get_repath_interval");

	ClassDB::bind_method(D_METHOD("set_simplify_path", "enable"), &AdvancedAINavigator3D::set_simplify_path);
	ClassDB::bind_method(D_METHOD("is_simplify_path"), &AdvancedAINavigator3D::is_simplify_path);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "simplify_path"), "set_simplify_path", "is_simplify_path");

	ClassDB::bind_method(D_METHOD("set_simplify_epsilon", "epsilon"), &AdvancedAINavigator3D::set_simplify_epsilon);
	ClassDB::bind_method(D_METHOD("get_simplify_epsilon"), &AdvancedAINavigator3D::get_simplify_epsilon);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "simplify_epsilon", PROPERTY_HINT_RANGE, "0.0,2.0,0.05"), "set_simplify_epsilon", "get_simplify_epsilon");

	ADD_GROUP("Movement", "");
	ClassDB::bind_method(D_METHOD("set_max_speed", "speed"), &AdvancedAINavigator3D::set_max_speed);
	ClassDB::bind_method(D_METHOD("get_max_speed"), &AdvancedAINavigator3D::get_max_speed);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_speed", PROPERTY_HINT_RANGE, "0.5,80.0,0.5"), "set_max_speed", "get_max_speed");

	ClassDB::bind_method(D_METHOD("set_acceleration", "accel"), &AdvancedAINavigator3D::set_acceleration);
	ClassDB::bind_method(D_METHOD("get_acceleration"), &AdvancedAINavigator3D::get_acceleration);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "acceleration", PROPERTY_HINT_RANGE, "1.0,100.0,0.5"), "set_acceleration", "get_acceleration");

	ClassDB::bind_method(D_METHOD("set_turn_speed", "speed"), &AdvancedAINavigator3D::set_turn_speed);
	ClassDB::bind_method(D_METHOD("get_turn_speed"), &AdvancedAINavigator3D::get_turn_speed);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "turn_speed", PROPERTY_HINT_RANGE, "0.5,30.0,0.5"), "set_turn_speed", "get_turn_speed");

	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &AdvancedAINavigator3D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &AdvancedAINavigator3D::get_gravity);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gravity", PROPERTY_HINT_RANGE, "0.0,40.0,0.1"), "set_gravity", "get_gravity");

	ADD_GROUP("Avoidance (RVO)", "");
	ClassDB::bind_method(D_METHOD("set_avoidance_enabled", "enable"), &AdvancedAINavigator3D::set_avoidance_enabled);
	ClassDB::bind_method(D_METHOD("is_avoidance_enabled"), &AdvancedAINavigator3D::is_avoidance_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "avoidance_enabled"), "set_avoidance_enabled", "is_avoidance_enabled");

	ClassDB::bind_method(D_METHOD("set_agent_radius", "radius"), &AdvancedAINavigator3D::set_agent_radius);
	ClassDB::bind_method(D_METHOD("get_agent_radius"), &AdvancedAINavigator3D::get_agent_radius);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "agent_radius", PROPERTY_HINT_RANGE, "0.1,10.0,0.05"), "set_agent_radius", "get_agent_radius");

	ClassDB::bind_method(D_METHOD("set_agent_height", "height"), &AdvancedAINavigator3D::set_agent_height);
	ClassDB::bind_method(D_METHOD("get_agent_height"), &AdvancedAINavigator3D::get_agent_height);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "agent_height", PROPERTY_HINT_RANGE, "0.2,10.0,0.1"), "set_agent_height", "get_agent_height");

	ClassDB::bind_method(D_METHOD("set_neighbor_distance", "distance"), &AdvancedAINavigator3D::set_neighbor_distance);
	ClassDB::bind_method(D_METHOD("get_neighbor_distance"), &AdvancedAINavigator3D::get_neighbor_distance);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "neighbor_distance", PROPERTY_HINT_RANGE, "1.0,50.0,0.5"), "set_neighbor_distance", "get_neighbor_distance");

	ClassDB::bind_method(D_METHOD("set_max_neighbors", "count"), &AdvancedAINavigator3D::set_max_neighbors);
	ClassDB::bind_method(D_METHOD("get_max_neighbors"), &AdvancedAINavigator3D::get_max_neighbors);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_neighbors", PROPERTY_HINT_RANGE, "1,64,1"), "set_max_neighbors", "get_max_neighbors");

	ClassDB::bind_method(D_METHOD("set_avoidance_layers", "layers"), &AdvancedAINavigator3D::set_avoidance_layers);
	ClassDB::bind_method(D_METHOD("get_avoidance_layers"), &AdvancedAINavigator3D::get_avoidance_layers);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "avoidance_layers", PROPERTY_HINT_LAYERS_AVOIDANCE), "set_avoidance_layers", "get_avoidance_layers");

	ClassDB::bind_method(D_METHOD("set_avoidance_mask", "mask"), &AdvancedAINavigator3D::set_avoidance_mask);
	ClassDB::bind_method(D_METHOD("get_avoidance_mask"), &AdvancedAINavigator3D::get_avoidance_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "avoidance_mask", PROPERTY_HINT_LAYERS_AVOIDANCE), "set_avoidance_mask", "get_avoidance_mask");

	ADD_GROUP("Dynamic Whisker Steering", "");
	ClassDB::bind_method(D_METHOD("set_local_whiskers_enabled", "enable"), &AdvancedAINavigator3D::set_local_whiskers_enabled);
	ClassDB::bind_method(D_METHOD("is_local_whiskers_enabled"), &AdvancedAINavigator3D::is_local_whiskers_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "local_whiskers_enabled"), "set_local_whiskers_enabled", "is_local_whiskers_enabled");

	ClassDB::bind_method(D_METHOD("set_obstacle_collision_mask", "mask"), &AdvancedAINavigator3D::set_obstacle_collision_mask);
	ClassDB::bind_method(D_METHOD("get_obstacle_collision_mask"), &AdvancedAINavigator3D::get_obstacle_collision_mask);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "obstacle_collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_obstacle_collision_mask", "get_obstacle_collision_mask");

	ClassDB::bind_method(D_METHOD("set_whisker_length", "length"), &AdvancedAINavigator3D::set_whisker_length);
	ClassDB::bind_method(D_METHOD("get_whisker_length"), &AdvancedAINavigator3D::get_whisker_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "whisker_length", PROPERTY_HINT_RANGE, "0.5,25.0,0.25"), "set_whisker_length", "get_whisker_length");

	ClassDB::bind_method(D_METHOD("set_whisker_spread_deg", "degrees"), &AdvancedAINavigator3D::set_whisker_spread_deg);
	ClassDB::bind_method(D_METHOD("get_whisker_spread_deg"), &AdvancedAINavigator3D::get_whisker_spread_deg);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "whisker_spread_deg", PROPERTY_HINT_RANGE, "10.0,120.0,1.0"), "set_whisker_spread_deg", "get_whisker_spread_deg");

	ClassDB::bind_method(D_METHOD("set_whisker_avoid_weight", "weight"), &AdvancedAINavigator3D::set_whisker_avoid_weight);
	ClassDB::bind_method(D_METHOD("get_whisker_avoid_weight"), &AdvancedAINavigator3D::get_whisker_avoid_weight);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "whisker_avoid_weight", PROPERTY_HINT_RANGE, "0.0,5.0,0.1"), "set_whisker_avoid_weight", "get_whisker_avoid_weight");

	// Signals
	ADD_SIGNAL(MethodInfo("path_updated", PropertyInfo(Variant::PACKED_VECTOR3_ARRAY, "path")));
	ADD_SIGNAL(MethodInfo("target_reached"));
	ADD_SIGNAL(MethodInfo("waypoint_reached", PropertyInfo(Variant::VECTOR3, "position"), PropertyInfo(Variant::INT, "index")));
	ADD_SIGNAL(MethodInfo("navigation_link_reached", PropertyInfo(Variant::VECTOR3, "link_position"), PropertyInfo(Variant::INT, "owner_id")));
	ADD_SIGNAL(MethodInfo("velocity_computed", PropertyInfo(Variant::VECTOR3, "safe_velocity")));
	ADD_SIGNAL(MethodInfo("agent_stuck", PropertyInfo(Variant::FLOAT, "stuck_duration")));
}

void AdvancedAINavigator3D::_notification(int p_what) {
	if (p_what == NOTIFICATION_EXIT_TREE) {
		_free_server_agent();
	} else if (p_what == NOTIFICATION_ENTER_TREE) {
		if (!Engine::get_singleton()->is_editor_hint() && !agent_rid.is_valid()) {
			_create_server_agent();
		}
	}
}

void AdvancedAINavigator3D::_ready() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	_create_server_agent();
	stuck_anchor_pos = get_global_position();
}

void AdvancedAINavigator3D::_create_server_agent() {
	if (agent_rid.is_valid()) {
		return;
	}
	NavigationServer3D *nav = NavigationServer3D::get_singleton();
	if (!nav) {
		return;
	}

	agent_rid = nav->agent_create();
	_sync_agent_properties();
}

void AdvancedAINavigator3D::_free_server_agent() {
	if (!agent_rid.is_valid()) {
		return;
	}
	NavigationServer3D *nav = NavigationServer3D::get_singleton();
	if (nav) {
		nav->agent_set_avoidance_callback(agent_rid, Callable());
		nav->free_rid(agent_rid);
	}
	agent_rid = RID();
}

RID AdvancedAINavigator3D::_get_active_map_rid() const {
	if (custom_map_rid.is_valid()) {
		return custom_map_rid;
	}
	Ref<World3D> world = get_world_3d();
	if (world.is_valid()) {
		return world->get_navigation_map();
	}
	return RID();
}

void AdvancedAINavigator3D::_sync_agent_properties() {
	if (!agent_rid.is_valid()) {
		return;
	}
	NavigationServer3D *nav = NavigationServer3D::get_singleton();
	if (!nav) {
		return;
	}

	RID map_rid = _get_active_map_rid();
	if (map_rid.is_valid()) {
		nav->agent_set_map(agent_rid, map_rid);
	}
	nav->agent_set_radius(agent_rid, agent_radius);
	nav->agent_set_height(agent_rid, agent_height);
	nav->agent_set_max_speed(agent_rid, max_speed);
	nav->agent_set_neighbor_distance(agent_rid, neighbor_distance);
	nav->agent_set_max_neighbors(agent_rid, max_neighbors);
	nav->agent_set_time_horizon_agents(agent_rid, time_horizon_agents);
	nav->agent_set_time_horizon_obstacles(agent_rid, time_horizon_obstacles);
	nav->agent_set_avoidance_layers(agent_rid, avoidance_layers);
	nav->agent_set_avoidance_mask(agent_rid, avoidance_mask);
	nav->agent_set_use_3d_avoidance(agent_rid, false);
	nav->agent_set_avoidance_enabled(agent_rid, avoidance_enabled);

	if (avoidance_enabled) {
		nav->agent_set_avoidance_callback(agent_rid, callable_mp(this, &AdvancedAINavigator3D::_on_avoidance_velocity_computed));
	} else {
		nav->agent_set_avoidance_callback(agent_rid, Callable());
	}
}

void AdvancedAINavigator3D::_on_avoidance_velocity_computed(const Vector3 &p_safe_velocity) {
	rvo_safe_velocity = p_safe_velocity;
	has_rvo_velocity = true;
	emit_signal("velocity_computed", p_safe_velocity);
}

void AdvancedAINavigator3D::set_target_position(const Vector3 &p_target) {
	target_position = p_target;
	navigation_active = true;
	if (target_position.distance_to(last_queried_target) >= repath_target_threshold || current_path.is_empty()) {
		force_repath();
	}
}

void AdvancedAINavigator3D::stop_navigation() {
	navigation_active = false;
	current_path.clear();
	current_waypoint_idx = 0;
	desired_velocity = Vector3();
	steer_hint = 0.0f;
	throttle_hint = 0.0f;
}

bool AdvancedAINavigator3D::is_navigation_finished() const {
	if (!navigation_active) {
		return true;
	}
	if (current_path.is_empty()) {
		return false;
	}
	return current_waypoint_idx >= current_path.size();
}

bool AdvancedAINavigator3D::force_repath() {
	NavigationServer3D *nav = NavigationServer3D::get_singleton();
	if (!nav) {
		return false;
	}

	RID map_rid = _get_active_map_rid();
	if (!map_rid.is_valid() || nav->map_get_iteration_id(map_rid) == 0) {
		// Navigation map hasn't finished its first physics sync yet.
		return false;
	}

	Vector3 origin = get_global_position();
	query_params->set_map(map_rid);
	query_params->set_start_position(origin);
	query_params->set_target_position(target_position);
	query_params->set_navigation_layers(navigation_layers);
	query_params->set_path_postprocessing(NavigationPathQueryParameters3D::PATH_POSTPROCESSING_CORRIDORFUNNEL);
	query_params->set_metadata_flags(NavigationPathQueryParameters3D::PATH_METADATA_INCLUDE_ALL);
	query_params->set_simplify_path(simplify_path);
	query_params->set_simplify_epsilon(simplify_epsilon);

	query_result->reset();
	nav->query_path(query_params, query_result);

	current_path = query_result->get_path();
	last_queried_target = target_position;
	repath_timer = 0.0;
	last_notified_link_idx = -1;

	if (current_path.is_empty()) {
		current_waypoint_idx = 0;
		return false;
	}

	// Skip initial point if we're already sitting right on it.
	current_waypoint_idx = (current_path.size() > 1 && origin.distance_to(current_path[0]) < waypoint_reached_distance * 0.5f) ? 1 : 0;

	emit_signal("path_updated", current_path);
	return true;
}

Vector3 AdvancedAINavigator3D::_compute_whisker_steering(const Vector3 &p_desired_dir, const TypedArray<RID> &p_exclude) {
	if (!local_whiskers_enabled || p_desired_dir.length_squared() < 0.0001f) {
		return p_desired_dir;
	}

	Ref<World3D> world = get_world_3d();
	if (world.is_null()) {
		return p_desired_dir;
	}
	PhysicsDirectSpaceState3D *space = world->get_direct_space_state();
	if (!space) {
		return p_desired_dir;
	}

	Vector3 flat_dir = Vector3(p_desired_dir.x, 0.0f, p_desired_dir.z).normalized();
	Vector3 origin = get_global_position() + Vector3(0.0f, whisker_height_offset, 0.0f);

	// 5 whiskers: center, +-half spread, +-full spread
	const float spread_rad = Math::deg_to_rad(whisker_spread_deg);
	const float angles[5] = {
		0.0f,
		spread_rad * 0.5f,
		-spread_rad * 0.5f,
		spread_rad,
		-spread_rad,
	};

	Vector3 avoidance_push = Vector3();
	float min_hit_ratio = 1.0f;

	for (int i = 0; i < 5; ++i) {
		Vector3 ray_dir = flat_dir.rotated(Vector3(0.0f, 1.0f, 0.0f), angles[i]);
		float len = (i == 0) ? whisker_length : whisker_length * 0.8f;
		Ref<PhysicsRayQueryParameters3D> ray = PhysicsRayQueryParameters3D::create(
			origin,
			origin + ray_dir * len,
			obstacle_collision_mask,
			p_exclude
		);

		Dictionary hit = space->intersect_ray(ray);
		if (!hit.is_empty()) {
			Vector3 hit_pos = hit["position"];
			float dist = origin.distance_to(hit_pos);
			float proximity = 1.0f - Math::clamp(dist / len, 0.0f, 1.0f);
			if (dist / len < min_hit_ratio) {
				min_hit_ratio = dist / len;
			}

			// Steer away from blocked ray; break symmetry if center whisker hits dead-on.
			Vector3 push_dir;
			if (i == 0) {
				push_dir = flat_dir.cross(Vector3(0.0f, 1.0f, 0.0f)) * static_cast<float>(recovery_dir_sign);
			} else {
				push_dir = -ray_dir;
			}
			avoidance_push += push_dir * (proximity * whisker_avoid_weight);
		}
	}

	Vector3 combined = flat_dir + avoidance_push;
	combined.y = 0.0f;
	if (combined.length_squared() < 0.001f) {
		// Dead-end: gently push laterally.
		combined = flat_dir.cross(Vector3(0.0f, 1.0f, 0.0f)) * static_cast<float>(recovery_dir_sign);
	}
	return combined.normalized() * Math::lerp(0.4f, 1.0f, min_hit_ratio);
}

void AdvancedAINavigator3D::_update_vehicle_hints(const Vector3 &p_move_dir, float p_speed_ratio) {
	if (recovery_reverse_timer > 0.0) {
		throttle_hint = -0.75f;
		steer_hint = static_cast<float>(recovery_dir_sign) * 0.9f;
		return;
	}

	if (p_move_dir.length_squared() < 0.0001f) {
		steer_hint = 0.0f;
		throttle_hint = 0.0f;
		return;
	}

	Node3D *basis_node = Object::cast_to<Node3D>(get_parent());
	if (!basis_node) {
		basis_node = this;
	}

	// Godot standard forward is -Z.
	Vector3 forward = -basis_node->get_global_transform().basis.get_column(2);
	forward.y = 0.0f;
	if (forward.length_squared() < 0.0001f) {
		forward = Vector3(0.0f, 0.0f, -1.0f);
	} else {
		forward.normalize();
	}

	Vector3 target_dir = Vector3(p_move_dir.x, 0.0f, p_move_dir.z).normalized();
	float dot_f = Math::clamp(forward.dot(target_dir), -1.0f, 1.0f);
	float cross_y = forward.cross(target_dir).y; // > 0 means target is to the left in Godot (+Y up)
	float signed_angle = std::atan2(cross_y, dot_f);

	steer_hint = Math::clamp(signed_angle / (static_cast<float>(Math_PI) * 0.45f), -1.0f, 1.0f);
	// Slow down into sharp turns, or reverse slightly if target is almost behind in vehicle mode.
	if (steering_mode == STEERING_VEHICLE && dot_f < -0.55f) {
		throttle_hint = -0.6f;
		steer_hint = (cross_y >= 0.0f) ? -1.0f : 1.0f;
	} else {
		float turn_slowdown = Math::lerp(0.45f, 1.0f, Math::clamp((dot_f + 1.0f) * 0.5f, 0.0f, 1.0f));
		throttle_hint = Math::clamp(p_speed_ratio * turn_slowdown, 0.0f, 1.0f);
	}
}

void AdvancedAINavigator3D::_update_stuck_detector(double p_delta, const Vector3 &p_pos) {
	if (recovery_reverse_timer > 0.0) {
		recovery_reverse_timer -= p_delta;
		return;
	}

	stuck_timer += p_delta;
	if (stuck_timer >= stuck_check_time) {
		float moved = p_pos.distance_to(stuck_anchor_pos);
		stuck_anchor_pos = p_pos;
		stuck_timer = 0.0;

		if (moved < stuck_min_progress) {
			emit_signal("agent_stuck", static_cast<float>(stuck_check_time));
			recovery_dir_sign = -recovery_dir_sign;
			recovery_reverse_timer = 0.65;
			force_repath();
		}
	}
}

void AdvancedAINavigator3D::_physics_process(double p_delta) {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	Vector3 current_pos = get_global_position();
	NavigationServer3D *nav = NavigationServer3D::get_singleton();
	if (nav && agent_rid.is_valid()) {
		nav->agent_set_position(agent_rid, current_pos);
	}

	if (!navigation_active) {
		return;
	}

	// Periodic repath (handles dynamic targets & map updates cheaply).
	repath_timer += p_delta;
	if (repath_timer >= repath_interval || current_path.is_empty()) {
		force_repath();
	}

	if (current_path.is_empty() || current_waypoint_idx >= current_path.size()) {
		desired_velocity = Vector3();
		steer_hint = 0.0f;
		throttle_hint = 0.0f;
		return;
	}

	// Advance waypoints & check NavigationLink3D metadata.
	while (current_waypoint_idx < current_path.size()) {
		Vector3 wp = current_path[current_waypoint_idx];
		float threshold = (current_waypoint_idx == current_path.size() - 1) ? target_reached_distance : waypoint_reached_distance;

		Vector3 flat_delta = wp - current_pos;
		flat_delta.y = 0.0f;

		if (flat_delta.length() <= threshold) {
			emit_signal("waypoint_reached", wp, current_waypoint_idx);

			// Check if this waypoint enters a NavigationLink3D.
			if (current_waypoint_idx != last_notified_link_idx && query_result.is_valid()) {
				PackedInt32Array types = query_result->get_path_types();
				if (current_waypoint_idx < types.size() && types[current_waypoint_idx] == NavigationPathQueryResult3D::PATH_SEGMENT_TYPE_LINK) {
					PackedInt64Array owners = query_result->get_path_owner_ids();
					int64_t owner_id = (current_waypoint_idx < owners.size()) ? owners[current_waypoint_idx] : 0;
					last_notified_link_idx = current_waypoint_idx;
					emit_signal("navigation_link_reached", wp, owner_id);
				}
			}

			current_waypoint_idx++;
			if (current_waypoint_idx >= current_path.size()) {
				stop_navigation();
				emit_signal("target_reached");
				return;
			}
		} else {
			break;
		}
	}

	_update_stuck_detector(p_delta, current_pos);

	Vector3 next_wp = current_path[current_waypoint_idx];
	Vector3 to_wp = next_wp - current_pos;
	to_wp.y = 0.0f;
	Vector3 raw_dir = (to_wp.length_squared() > 0.0001f) ? to_wp.normalized() : Vector3();

	// Exclude parent CollisionObject3D (e.g. CharacterBody3D or car RigidBody3D) from whisker rays.
	TypedArray<RID> exclude;
	if (CollisionObject3D *col = Object::cast_to<CollisionObject3D>(get_parent())) {
		exclude.push_back(col->get_rid());
	}

	Vector3 steered_vec = _compute_whisker_steering(raw_dir, exclude);
	float speed_scale = Math::clamp(steered_vec.length(), 0.0f, 1.0f);
	Vector3 steered_dir = (steered_vec.length_squared() > 0.0001f) ? steered_vec.normalized() : raw_dir;

	if (recovery_reverse_timer > 0.0 && steering_mode == STEERING_OMNI) {
		steered_dir = (steered_dir.cross(Vector3(0.0f, 1.0f, 0.0f)) * static_cast<float>(recovery_dir_sign) - raw_dir * 0.5f).normalized();
	}

	desired_velocity = steered_dir * (max_speed * speed_scale);
	_update_vehicle_hints(steered_dir, speed_scale);

	if (nav && agent_rid.is_valid() && avoidance_enabled) {
		nav->agent_set_velocity(agent_rid, desired_velocity);
	}

	Vector3 target_vel = (avoidance_enabled && has_rvo_velocity) ? rvo_safe_velocity : desired_velocity;

	// Optional automatic movement if parent is a CharacterBody3D.
	if (auto_move_parent_body) {
		if (CharacterBody3D *body = Object::cast_to<CharacterBody3D>(get_parent())) {
			Vector3 vel = body->get_velocity();
			Vector3 horiz = Vector3(vel.x, 0.0f, vel.z);
			horiz = horiz.move_toward(Vector3(target_vel.x, 0.0f, target_vel.z), acceleration * static_cast<float>(p_delta));

			vel.x = horiz.x;
			vel.z = horiz.z;
			if (!body->is_on_floor()) {
				vel.y -= gravity * static_cast<float>(p_delta);
			}

			body->set_velocity(vel);
			body->move_and_slide();

			if (horiz.length_squared() > 0.05f) {
				Vector3 look_dir = horiz.normalized();
				float target_yaw = std::atan2(-look_dir.x, -look_dir.z);
				Vector3 rot = body->get_rotation();
				rot.y = Math::lerp_angle(rot.y, target_yaw, Math::clamp(turn_speed * static_cast<float>(p_delta), 0.0f, 1.0f));
				body->set_rotation(rot);
			}
		}
	}
}

Vector3 AdvancedAINavigator3D::get_next_waypoint() const {
	if (current_path.is_empty() || current_waypoint_idx >= current_path.size()) {
		return get_global_position();
	}
	return current_path[current_waypoint_idx];
}

float AdvancedAINavigator3D::get_remaining_distance() const {
	if (current_path.is_empty() || current_waypoint_idx >= current_path.size()) {
		return 0.0f;
	}
	float dist = get_global_position().distance_to(current_path[current_waypoint_idx]);
	for (int32_t i = current_waypoint_idx; i < current_path.size() - 1; ++i) {
		dist += current_path[i].distance_to(current_path[i + 1]);
	}
	return dist;
}

void AdvancedAINavigator3D::set_custom_navigation_map(const RID &p_map) {
	custom_map_rid = p_map;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_max_speed(float p_speed) {
	max_speed = p_speed;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_avoidance_enabled(bool p_enable) {
	avoidance_enabled = p_enable;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_agent_radius(float p_radius) {
	agent_radius = p_radius;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_agent_height(float p_height) {
	agent_height = p_height;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_neighbor_distance(float p_dist) {
	neighbor_distance = p_dist;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_max_neighbors(int32_t p_max) {
	max_neighbors = p_max;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_avoidance_layers(uint32_t p_layers) {
	avoidance_layers = p_layers;
	_sync_agent_properties();
}

void AdvancedAINavigator3D::set_avoidance_mask(uint32_t p_mask) {
	avoidance_mask = p_mask;
	_sync_agent_properties();
}