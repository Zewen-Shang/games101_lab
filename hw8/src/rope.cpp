#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D dis = (end - start)/(num_nodes-1);
        for(int i=0;i<num_nodes;i++){
            masses.push_back(new Mass(start + i * dis,node_mass,false));
        }
//        Comment-in this part when you implement the constructor
       for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
        }

        for(int i=0;i<num_nodes-1;i++){
            springs.push_back(new Spring(masses[i],masses[i+1],k));
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D a = s->m1->position,b = s->m2->position;
            Vector2D fba = -s->k * (b-a)/(b-a).norm() * ((b-a).norm() - s->rest_length);
            Vector2D fab = -fba;
            s->m1->forces += fab;
            s->m2->forces += fba;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += gravity* m->mass;
                // TODO (Part 2): Add global damping
                Vector2D a = m->forces/m->mass;
                Vector2D vt = m->velocity;
                m->velocity = vt + delta_t * a;
                m->position = m->position + m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D a = s->m1->position,b = s->m2->position;
            Vector2D fba = -s->k * (b-a)/(b-a).norm() * ((b-a).norm() - s->rest_length);
            Vector2D fab = -fba;
            s->m1->forces += fab;
            s->m2->forces += fba;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces += gravity * m->mass;
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D a = m->forces/m->mass;
                float dump = 0.00005;
                m->position = temp_position + (1-dump)*(temp_position - m->last_position) + a * delta_t * delta_t;
                // TODO (Part 4): Add global Verlet damping
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0,0);
        }
    }
}
