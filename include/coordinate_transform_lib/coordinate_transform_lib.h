#ifndef COORDINATE_TRANSFORM_LIB_H
#define COORDINATE_TRANSFORM_LIB_H

#include <string>
#include <unordered_map>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"          
#include "tf2_ros/transform_listener.h"
#include <yaml-cpp/yaml.h>

/**
 * @brief Перечисление для обозначения статусов при проверке границ.
 */

enum class ResultStatus {
    SUCCESS,
    OUT_OF_BOUNDS,
    TRANSFORM_NOT_FOUND,
    INVALID_INPUT
};

/**
 * @brief Класс для преобразования координат между системами отсчёта с учётом смещения, 
 * ориентации и проверки границ.
 */

class CoordinateTransformer {
    public:
    /**
     * @brief Конструктор CoordinateTransformer
     */
    CoordinateTransformer(rclcpp::Node::SharedPtr node);  
    /**
     * @brief метод для прямого преобразования координат из исходного пространства в целевое
     */
    ResultStatus convert(
        const geometry_msgs::msg::PoseStamped& input,
        geometry_msgs::msg::PoseStamped& output,
        const std::string& target_frame
    );
    /**
     * @brief метод для обратного преобразования координат из целевого пространства в исходное
     */ 
    ResultStatus inverseConvert(
        const geometry_msgs::msg::PoseStamped& input,
        geometry_msgs::msg::PoseStamped& output,
        const std::string& source_frame
    );
        
        /**
         * @brief метод для загрузки трансформации из YAML-файла
         * 
         */ 
        bool loadTransformsFromYAML(const std::string& yaml_file);
        /**
         * @brief метод для добавления трансформации вручную
         * 
         */ 
        void addTransform(const std::string& parent_frame,
                          const std::string& child_frame,
                          const tf2::Vector3DoubleData& translation,
                          const tf2::Quaternion& rotation);
        /**
         * @brief метод для прямого преобразование координат 
         * из родительской в дочернюю систему отсчета
         * 
         */ 
        tf2::Vector3 transformPoint(const std::string& from_frame,
                                    const std::string& to_frame,
                                    const tf2::Vector3DoubleData& point) const;
        /**
         * @brief метод для проверки возможности трансформации между системами
         * 
         */ 
        bool canTransform(const std::string& from_frame, const std::string& to_frame) const;

        /**
         * @brief метод для проверки границ
         * 
         */ 
        void setBounds(
            const std::string& frame_id,
            const geometry_msgs::msg::Point& min,
            const geometry_msgs::msg::Point& max
           );
        /**
         * @brief метод для для вычисления преобразования координат между различными системами отсчета
         * 
         */ 
        bool computeTransform(const std::string& from_frame,
            const std::string& to_frame,
            tf2::Transform& out_transform) const;
    
    private:
   
        tf2_ros::Buffer tf_buffer_;                      // Объявление tf_buffer_
        tf2_ros::TransformListener tf_listener_;         // Объявление tf_listener_
        rclcpp::Node::SharedPtr node_;   

        // структура для хранения трансформаций
        struct Transform {
            std::string parent;
            tf2::Transform tf;
        };

        // Структура для хранения границ
        struct Bounds {
            geometry_msgs::msg::Point min;
            geometry_msgs::msg::Point max;
        };
    
        // map: имя дочернего фрейма -> его трансформация
        std::unordered_map<std::string, Transform> transforms_;
        
        // Map: имя фрейма -> его границы
        std::unordered_map<std::string, Bounds> bounds_;
    
    };

#endif //COORDINATE_TRANSFORM_LIB_H