#include "coordinate_transform_lib/coordinate_transform_lib.h"
#include <vector>
#include <iostream>
#include <stdexcept>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/Vector3.h"

using namespace std;
using namespace geometry_msgs;

// здезсь пишем реализацию методов, что были определены в .h файле
//либа уже подключена, вызвать ее в мейн файле

CoordinateTransformer::CoordinateTransformer(rclcpp::Node::SharedPtr node)
	: tf_buffer_(node->get_clock()),
	tf_listener_(tf_buffer_){
}


ResultStatus CoordinateTransformer::convert(
	const geometry_msgs::msg::PoseStamped& input,
	geometry_msgs::msg::PoseStamped&       output,
	const string&                          target_frame)
{
	// проверка входных данных на подлинность
	if (input.header.frame_id.empty()) {
		return ResultStatus::INVALID_INPUT;
	}

	try {
		// если временная метка отсутствует - используем текущее время
		rclcpp::Time time_to_use = input.header.stamp;
		if (time_to_use == rclcpp::Time()) {
			rclcpp::Clock clock;
			time_to_use = clock.now();
		}

		// получаем трансформацию для заданно цели
		geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
			target_frame, input.header.frame_id, time_to_use);

		// применяем трансформацию к входному PoseStamped и устанавливаем временную метку
		tf2::doTransform(input, output, transform);
		output.header.frame_id = target_frame;
		output.header.stamp = time_to_use;

		return ResultStatus::SUCCESS;
	}
	catch (tf2::TransformException &ex) {
		RCLCPP_WARN(rclcpp::get_logger("CoordinateTransformer"), "Transform error: %s", ex.what());
		return ResultStatus::TRANSFORM_NOT_FOUND;
	}
}


ResultStatus CoordinateTransformer::inverseConvert(
	const geometry_msgs::msg::PoseStamped& input,
	geometry_msgs::msg::PoseStamped&       output,
	const string&                          source_frame)
{
	// проверка на валидность входных данных
	if (input.header.frame_id.empty()) {
		return ResultStatus::INVALID_INPUT;
	}

	try {
		// если временная метка отсутствует, используем текущее время
		rclcpp::Time time_to_use = input.header.stamp;
		if (time_to_use == rclcpp::Time()) {
			rclcpp::Clock clock;
			time_to_use = clock.now();
		}

		// получаем трансформацию для обратного преобразования
		geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
			input.header.frame_id, source_frame, time_to_use);

		// применяем обратную трансформацию к входному PoseStamped
		tf2::doTransform(input, output, transform);
		output.header.frame_id = source_frame;
		output.header.stamp = time_to_use;

		return ResultStatus::SUCCESS;
	}
	catch (tf2::TransformException &ex) {
		RCLCPP_WARN(rclcpp::get_logger("CoordinateTransformer"), "Transform error: %s", ex.what());
		return ResultStatus::TRANSFORM_NOT_FOUND;
	}
}

// добавление трансформации вручную
void CoordinateTransformer::addTransform(const string&            parent_frame,
                                         const string&            child_frame,
                                         const Vector3DoubleData& translation,
                                         const Quaternion&        rotation) {
	tf2::Transform transform(rotation, translation);
	transforms_[child_frame] = {parent_frame, transform};
}

// загрузка трансформации из YAML - файла
bool CoordinateTransformer::loadTransformsFromYAML(const std::string& yaml_file) {
	try {
		YAML::Node config = YAML::LoadFile(yaml_file);

		for (const auto& node : config) {
			string parent = node["parent_frame"].as<string>();
			string child = node["child_frame"].as<string>();

			auto translation_node = node["translation"];
			auto rotation_node = node["rotation"];

			tf2::Vector3 translation(
				translation_node["x"].as<double>(),
				translation_node["y"].as<double>(),
				translation_node["z"].as<double>());

			tf2::Quaternion rotation(
				rotation_node["x"].as<double>(),
				rotation_node["y"].as<double>(),
				rotation_node["z"].as<double>(),
				rotation_node["w"].as<double>());

			addTransform(parent, child, translation, rotation);
		}
		return true;
	} catch (const exception& e) {
		cerr << "Error loaded YAML: " << e.what() << endl;
		return false;
	}
}



// проверка возможности трансформации между фреймами
bool CoordinateTransformer::canTransform(const string& from_frame, const string& to_frame) const {
	tf2::Transform dummy;
	return computeTransform(from_frame, to_frame, dummy);
}

// прямое или обратное преобразование точки
tf2::Vector3 CoordinateTransformer::transformPoint(const string&                 from_frame,
                                                   const string&                 to_frame,
                                                   const tf2::Vector3DoubleData& point) const {
	tf2::Transform transform;
	if (!computeTransform(from_frame, to_frame, transform)) {
		throw runtime_error("Transformation is not available: " + from_frame + " -> " + to_frame);
	}
	return transform * point;
}

// рекурсивное построение трансформации от from_frame до to_frame
bool CoordinateTransformer::computeTransform(const string&   from_frame,
                                             const string&   to_frame,
                                             tf2::Transform& out_transform) const {
	if (from_frame == to_frame) {
		out_transform.setIdentity();
		return true;
	}

	auto it = transforms_.find(from_frame);
	if (it == transforms_.end()) return false;

	const Transform& current = it->second;

	tf2::Transform parent_to_current = current.tf;
	tf2::Transform recursive_transform;

	if (!computeTransform(current.parent, to_frame, recursive_transform)) return false;

	out_transform = recursive_transform * parent_to_current;
	return true;
}
// проверка границ
void CoordinateTransformer::setBounds(
	const string&                    frame_id,
	const geometry_msgs::msg::Point& min,
	const geometry_msgs::msg::Point& max
	) {
	// сохраняем границы в map
	bounds_[frame_id] = {min, max};
}