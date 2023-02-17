#include "runtime/function/controller/character_controller.h"

#include "runtime/core/base/macro.h"

#include "runtime/function/framework/component/motor/motor_component.h"
#include "runtime/function/framework/world/world_manager.h"
#include "runtime/function/global/global_context.h"
#include "runtime/function/physics/physics_scene.h"

namespace Pilot
{
    CharacterController::CharacterController(const Capsule& capsule) : m_capsule(capsule)
    {
        m_rigidbody_shape                                    = RigidBodyShape();
        m_rigidbody_shape.m_geometry                         = PILOT_REFLECTION_NEW(Capsule);
        *static_cast<Capsule*>(m_rigidbody_shape.m_geometry) = m_capsule;

        m_rigidbody_shape.m_type = RigidBodyShapeType::capsule;

        Quaternion orientation;
        orientation.fromAngleAxis(Radian(Degree(90.f)), Vector3::UNIT_X);

        m_rigidbody_shape.m_local_transform =
            Transform(
                Vector3(0, 0, capsule.m_half_height + capsule.m_radius),
                orientation,
                Vector3::UNIT_SCALE);
    }

    Vector3 CharacterController::move(const Vector3& current_position, const Vector3& displacement)
    {
        std::shared_ptr<PhysicsScene> physics_scene =
            g_runtime_global_context.m_world_manager->getCurrentActivePhysicsScene().lock();
        ASSERT(physics_scene);

        std::vector<PhysicsHitInfo> hits;

        Transform world_transform = Transform(
            current_position + 0.1f * Vector3::UNIT_Z,
            Quaternion::IDENTITY,
            Vector3::UNIT_SCALE);

        Vector3 vertical_displacement   = displacement.z * Vector3::UNIT_Z;
        Vector3 horizontal_displacement = Vector3(displacement.x, displacement.y, 0.f);

        Vector3 vertical_direction   = vertical_displacement.normalisedCopy();
        Vector3 horizontal_direction = horizontal_displacement.normalisedCopy();

        Vector3 final_position = current_position;

        m_is_touch_ground = physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            Vector3::NEGATIVE_UNIT_Z,
            0.105f,
            hits);

        hits.clear();
        
        world_transform.m_position -= 0.1f * Vector3::UNIT_Z;

        // vertical pass
        if (physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            vertical_direction,
            vertical_displacement.length(),
            hits))
        {
            final_position += hits[0].hit_distance * vertical_direction;
        }
        else
        {
            final_position += vertical_displacement;
        }

        hits.clear();

        float maxStepSize = 0.5f;

        // side pass
        if (physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            horizontal_direction,
            horizontal_displacement.length(),
            hits))
        {
            float move_distance = Math::max(hits[0].hit_distance - 0.001f, 0.0f);
            float total_distance = move_distance;
            final_position += move_distance * horizontal_direction;
            
            Vector3 horizontal_displacement2 = horizontal_displacement - move_distance * horizontal_direction;
            //能够登上台阶
            if (hits[0].hit_position.z - final_position.z < maxStepSize)
            {
                std::vector<PhysicsHitInfo> hits2;
                if (physics_scene->raycast(  
                    final_position + 0.01f, 
                    horizontal_displacement2,
                    horizontal_displacement2.length(), 
                    hits2))
                {
                    float move_distance2 = Math::max(hits2[0].hit_distance - 0.01f, 0.0f);
                    total_distance += move_distance2;
                    horizontal_displacement2 = move_distance2 * horizontal_direction;
                }
                if (horizontal_displacement2.length() > 0.01f)
                {
                    final_position += (hits[0].hit_position.z - final_position.z) * Vector3::UNIT_Z +
                                      horizontal_displacement2;
                }
            }

            //沿着墙移动
            Vector3 dir = Vector3::UNIT_Z.crossProduct(hits[0].hit_normal);
            dir         = horizontal_direction.dotProduct(dir) > 0 ? dir : -dir;
            float distance = Math::max(horizontal_displacement.length() - total_distance - 0.001f, 0.0f);
            if (distance > 0.001f)
            {
                std::vector<PhysicsHitInfo> hits3;
                Transform world_transform3 = Transform(final_position, Quaternion::IDENTITY, Vector3::UNIT_SCALE);
                if (physics_scene->sweep(m_rigidbody_shape, world_transform3.getMatrix(), dir, distance, hits3))
                {
                    final_position += (hits3[0].hit_distance - 0.001f) * dir;
                }
                else
                {
                    final_position += distance * dir;
                }
            }
        }
        else
        {
            final_position += horizontal_displacement;
        }

        return final_position;
    }

} // namespace Pilot
