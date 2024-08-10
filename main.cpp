#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <thread>
#include <SFML/Graphics.hpp>

#include "vector_functions.h"

#define PI 3.14159

using namespace std;

// Window parameters

const int window_width = 1000;
const int window_height = 1000;

// Partition parameters
const int partition_width = 24;
const int partition_height = 24;

// Boundary parameters

Vector2 boundary_center = {window_width/2, window_height/2};
const float boundary_radius = 300;

// World parameters

const float gravity               = 1000.f;
const float energy_conservation   = .99f;
const float overshot_compensation = 1.1f;
const float drag                  = .99f;
const float draw_scale            = .05f;
const int   sub_updates           = 4;
const float spawn_speed           = .05f;
const int   particle_min_size     = 5;
const int   particle_max_size     = 10;


// Globals
vector<Particle*> particles {};
vector<Particle*> **partitions;

// Functions

Particle* initialize(float radius, float mass, Vector2 pos, Vector2 vel) {
    Particle *p = new Particle();
    (*p).radius = radius;
    (*p).mass = mass;
    (*p).position = pos;
    (*p).velocity = vel;
    return p;
}

void particle_print(Particle *p, char name[]) {
    printf("=========================================\n");
    printf("%s\n", name);
    printf("Radius: %f\n", (*p).radius);
    printf("Mass: %f\n", (*p).mass);
    printf("Position: (%f, %f)\n", (*p).position.x, (*p).position.y);
    printf("Velocity: (%f, %f)\n", (*p).velocity.x, (*p).velocity.y);
    Vector2 u = unit((*p).velocity);
    printf("Unit: (%f, %f)\n", u.x, u.y);
    float m = magnitude((*p).velocity);
    printf("Magnitude: %f\n", m);
    printf("Momentum: (%f, %f)\n", (*p).mass*(*p).velocity.x, (*p).mass*(*p).velocity.y);
    printf("=========================================\n");
}

void partition_clear() {
    for(int x = 0; x < partition_width; x++) {
        for(int y = 0; y < partition_height; y++) {
            partitions[x][y].clear();
        }
    }
}

void make_pool(int x, int y, vector<Particle*> *pool) {
    for(int i = -1; i < 2; i++) {
        for(int j = -1; j < 2; j++) {
            vector<Particle*> neighbor = partitions[x+i][y+j];
            (*pool).reserve((*pool).size() + neighbor.size());
            (*pool).insert((*pool).end(), neighbor.begin(), neighbor.end());
        }
    }
}

Vector2 position_to_partition(Vector2 position, Vector2 partition_size) {
    return (position - (boundary_center - Vector2{boundary_radius, boundary_radius}))/partition_size;
}

Vector2 partition_to_position(int x, int y, Vector2 partition_size) {
    return Vector2{(float)x, (float)y}*partition_size + (boundary_center - Vector2{boundary_radius, boundary_radius});
}

void particle_to_partition(Vector2 partition_size) {
    partition_clear();
    
    for(Particle *p : particles) {
        Vector2 pos = position_to_partition((*p).position, partition_size);
        partitions[(int)pos.x][(int)pos.y].push_back(p);
    }
}

// Restraint functions

void restrain_particle(Particle *p) {
    float dist = distance((*p).position, boundary_center);
        if(dist > boundary_radius - (*p).radius) {
            Vector2 norm = unit(boundary_center - (*p).position);
            float overshot = (dist - boundary_radius + (*p).radius);
            (*p).position = (*p).position + norm*overshot*overshot_compensation;
            (*p).velocity = reflect((*p).velocity, norm) * energy_conservation;
        }
}

void check_boundary(vector<Vector2> *boundary_partitions) {
    for(Vector2 partition : (*boundary_partitions)) {
        for(Particle *p : (partitions[(int)partition.x][(int)partition.y])) {
            float dist = distance((*p).position, boundary_center);
            if(dist > boundary_radius - (*p).radius) {
                Vector2 norm = unit(boundary_center - (*p).position);
                float overshot = (dist - boundary_radius + (*p).radius);
                (*p).position = (*p).position + norm*overshot*overshot_compensation;
                (*p).velocity = reflect((*p).velocity, norm) * energy_conservation;
            }
        }
    }
}

// https://stackoverflow.com/questions/35211114/2d-elastic-ball-collision-physics
Vector2 collision_formula(Particle *p1, Particle *p2) {
    return (*p1).velocity 
    - ((*p1).position-(*p2).position) 
    * dot((*p1).velocity-(*p2).velocity, (*p1).position-(*p2).position) / pow(magnitude((*p1).position-(*p2).position), 2) 
    * (2*(*p2).mass/((*p1).mass+(*p2).mass));
}

void check_collision_cluster(vector<Particle*> *pool) {
    for(Particle *p1 : *pool) {
        for(Particle *p2 : *pool) {
            if(p1 != p2) {
                float overshot = ((*p1).radius + (*p2).radius - distance((*p1).position, (*p2).position));
                if(overshot > 0) {
                    Vector2 norm;
                    if((*p1).position == (*p2).position) {
                        norm = unit({(float)(rand() % 101 - 50), (float)(rand() % 101 - 50)});
                    } else {
                        norm = unit((*p1).position - (*p2).position);
                    }

                    (*p1).position = (*p1).position + norm*overshot/2.f*overshot_compensation;
                    (*p2).position = (*p2).position - norm*overshot/2.f*overshot_compensation;

                    Vector2 p1_velocity_tmp = collision_formula(p1, p2);
                    Vector2 p2_velocity_tmp = collision_formula(p2, p1);

                    (*p1).velocity = p1_velocity_tmp * energy_conservation;
                    (*p2).velocity = p2_velocity_tmp * energy_conservation;
                }
            }
        }
    }
}

void collision_thread(int x1, int x2) {
    vector<Particle*> pool;
    for(int x = x1; x < x2; x++) {
        for(int y = 0; y < partition_height; y++) {
            if(partitions[x][y].size() > 0) {
                pool.clear();
                make_pool(x, y, &pool);
                check_collision_cluster(&pool);
            }
        }
    }
}

void check_collisions() {
    // thread t1(collision_thread, 0, partition_width/2);
    // thread t2(collision_thread, partition_width/2 + 1, partition_width);
    
    // t1.join();
    // t2.join();

    vector<Particle*> pool;
    for(int x = 1; x < partition_width-1; x++) {
        for(int y = 1; y < partition_height-1; y++) {
            if(partitions[x][y].size() > 0) {
                pool.clear();
                make_pool(x, y, &pool);
                check_collision_cluster(&pool);
            }
        }
    }
    
}

// Kinematics

void update_kinematics(float dt, bool rmb_toggle, Vector2 mouse_pos, bool gravity_toggle, float *energy, Vector2 partition_size) {
    partition_clear();
    for(Particle *p : particles) {
        // Kinematics
        Vector2 acceleration;
        if(rmb_toggle) {
            acceleration = unit(mouse_pos - (*p).position) * gravity * distance(mouse_pos, (*p).position) / 50;
        } else {
            acceleration = (gravity_toggle) ? Vector2{0.f, gravity} : Vector2{0.f, 0.f};
        }
        (*p).position = (*p).position + (*p).velocity*dt + acceleration*dt*dt/2.f;
        (*p).velocity = (*p).velocity + acceleration*dt;
        *energy += (*p).mass * pow(magnitude((*p).velocity), 2) / 2.f / 1000.f / sub_updates;

        // Restrain
        restrain_particle(p);

        // Partitioning
        Vector2 pos = position_to_partition((*p).position, partition_size);
        partitions[(int)pos.x][(int)pos.y].push_back(p);
    }
}

// Update

void update(float dt, bool rmb_toggle, Vector2 mouse_pos, bool gravity_toggle, float *energy, vector<Vector2> *boundary_partitions, Vector2 partition_size) {
    float sub_dt = dt/sub_updates;
    for(int i = 0; i < sub_updates; i++) {
        update_kinematics(sub_dt, rmb_toggle, mouse_pos, gravity_toggle, energy, partition_size);
        check_collisions();
        check_boundary(boundary_partitions);
    }
}

// Drawing

void draw_partitions(Vector2 partition_size, vector<Vector2> *boundary_partitions, sf::RenderWindow *window, sf::Font *font) {
    for(int x = 0; x < partition_width; x++) {
        for(int y = 0; y < partition_height; y++) {
            sf::RectangleShape rect(sf::Vector2f(partition_size.x-2, partition_size.y-2));
            rect.setOrigin(rect.getSize().x*.5f, rect.getSize().y*.5f);
            
            rect.setFillColor(sf::Color(100,100,100,50));
            Vector2 adjustment = partition_to_position(x, y, partition_size) + partition_size/2;
            rect.setPosition(adjustment.x, adjustment.y);

            (*window).draw(rect);
        }
    }

    for(Vector2 boundary_partition : *boundary_partitions) {
        sf::RectangleShape rect(sf::Vector2f(partition_size.x-2, partition_size.y-2));
        rect.setOrigin(rect.getSize().x*.5f, rect.getSize().y*.5f);
        
        rect.setFillColor(sf::Color(100,100,200,50));
        Vector2 adjustment = partition_to_position(boundary_partition.x, boundary_partition.y, partition_size) + partition_size/2;
        rect.setPosition(adjustment.x, adjustment.y);

        (*window).draw(rect);
    }
}

void draw_velocity(sf::RenderWindow *window) {
    for(Particle *p : particles) {
        float length = magnitude((*p).velocity);
        sf::RectangleShape rect(sf::Vector2f(length*draw_scale, 2));
        rect.setOrigin(rect.getSize().x*.5f, rect.getSize().y*.5f);
        rect.setFillColor(sf::Color::Red);
        rect.setPosition((*p).position.x + (*p).velocity.x*draw_scale/2.f, (*p).position.y + (*p).velocity.y*draw_scale/2.f);
        rect.setRotation(angle({0, 0}, (*p).velocity)*180.f/PI);

        (*window).draw(rect);
    }
}

void draw_particles(sf::RenderWindow *window) {
    for(Particle *p : particles) {
        sf::CircleShape shape((*p).radius);
        shape.setFillColor(sf::Color::Green);
        shape.setPosition((*p).position.x - (*p).radius, (*p).position.y - (*p).radius);

        (*window).draw(shape);
    }
}

// Setup text

void setup_text(sf::Text *txt, sf::Font *font) {
    (*txt).setString("/////");
    (*txt).setCharacterSize(24);
    (*txt).setStyle(sf::Text::Bold);
    (*txt).setFillColor(sf::Color::White);
    (*txt).setFont(*font);
}

void make_partition(int x, int y, vector<Vector2> *boundary_partitions, Vector2 partition_size, float radius) {
    Vector2 top_left = partition_to_position(x, y, partition_size);
    Vector2 top_right = partition_to_position(x, y, partition_size) + Vector2{partition_size.x, 0};
    Vector2 bottom_left = partition_to_position(x, y, partition_size) + Vector2{0, partition_size.y};
    Vector2 bottom_right = partition_to_position(x, y, partition_size) + partition_size;
    bool top_left_within = distance(top_left, boundary_center) <= radius;
    bool top_right_within = distance(top_right, boundary_center) <= radius;
    bool bottom_left_within = distance(bottom_left, boundary_center) <= radius;
    bool bottom_right_within = distance(bottom_right, boundary_center) <= radius;
    int corners_within = top_left_within + top_right_within + bottom_left_within + bottom_right_within;
    if(corners_within < 4 && corners_within > 0) {
        (*boundary_partitions).push_back(Vector2{(float)x, (float)y});
    }
}

int main()
{
    // Variables
    sf::RenderWindow window(sf::VideoMode(window_width, window_height), "particles");
    bool  pause_toggle     = false;
    bool  draw_toggle      = false;
    bool  gravity_toggle   = true;
    bool  lmb_toggle       = false;
    bool  rmb_toggle       = false;
    float spawn_speed_time = spawn_speed;
    float delta_time       = 0.f;
    int   fps              = 0;
    float energy           = 0.f;

    // Partitioning
    partitions = new vector<Particle*>*[partition_width];
    for(int i = 0; i < partition_width; i++) {
        partitions[i] = new vector<Particle*>[partition_height];
    }
    Vector2 partition_size = {boundary_radius*2/partition_width, boundary_radius*2/partition_height};
    vector<Vector2> boundary_partitions;
    for(int x = 0; x < partition_width; x++) {
        for(int y = 0; y < partition_height; y++) {
            make_partition(x, y, &boundary_partitions, partition_size, boundary_radius);
            make_partition(x, y, &boundary_partitions, partition_size, boundary_radius-partition_size.x*1.4f);
        }
    }

    // Boundary settings
    sf::CircleShape boundary(boundary_radius);
    boundary.setFillColor(sf::Color::White);
    boundary.setPosition(boundary_center.x - boundary_radius, boundary_center.y - boundary_radius);
    boundary.setPointCount(200);

    // Text settings
    sf::Font font;
    font.loadFromFile("arial.ttf");
    
    sf::Text particle_count_text;
    setup_text(&particle_count_text, &font);
    particle_count_text.setPosition(6, 0);

    sf::Text kinetic_energy_text;
    setup_text(&kinetic_energy_text, &font);
    kinetic_energy_text.setPosition(6, 30);

    sf::Text fps_text;
    setup_text(&fps_text, &font);
    fps_text.setPosition(6, 60);
    
    // FPS bar
    sf::RectangleShape fps_bar(sf::Vector2f(0, 4));
    fps_bar.setFillColor(sf::Color::Green);
    fps_bar.setPosition(6, 90);
    
    sf::Text controls_text;
    setup_text(&controls_text, &font);
    controls_text.setPosition(6, 880);
    controls_text.setCharacterSize(16);
    controls_text.setString(
        "spawn particle - LMB\n" 
        "gravitate mouse - RMB\n" 
        "toggle gravity - G\n" 
        "debug - D\n" 
        "clear - C\n" 
        "exit - ESC\n"
    );
    
    sf::Clock clock;

    while (window.isOpen())
    {
        sf::Event event;

        // Checking events
        while (window.pollEvent(event))
        {
            switch(event.type) {
                case sf::Event::Closed: {
                    window.close();
                    break;
                }
                case sf::Event::MouseButtonPressed: {
                    if (event.mouseButton.button == sf::Mouse::Left)
                        lmb_toggle = true;
                    if (event.mouseButton.button == sf::Mouse::Right)
                        rmb_toggle = true;
                    break;
                }
                case sf::Event::MouseButtonReleased: {
                    if (event.mouseButton.button == sf::Mouse::Left) {
                        lmb_toggle = false;
                        spawn_speed_time = spawn_speed;
                    }
                    if (event.mouseButton.button == sf::Mouse::Right)
                        rmb_toggle = false;
                    break;
                }
                case sf::Event::KeyPressed: {
                    switch(event.key.code) {
                        case sf::Keyboard::Escape: {
                            window.close();
                            break;
                        }
                        case sf::Keyboard::Space: {
                            pause_toggle = !pause_toggle ;
                            break;
                        }
                        case sf::Keyboard::D: {
                            draw_toggle = !draw_toggle;
                            break;
                        }
                        case sf::Keyboard::C: {
                            particles.clear();
                            break;
                        }
                        case sf::Keyboard::G: {
                            gravity_toggle = !gravity_toggle;
                            break;
                        }
                        default: {
                            break;
                        }
                    }
                    break;
                }
                default: {
                    break;
                }
            }
        }

        // Getting delta_time
        delta_time = clock.restart().asSeconds();

        // Particle spawner
        if(lmb_toggle) {
            if(spawn_speed_time >= spawn_speed) {
                sf::Vector2i mouse_position = sf::Mouse::getPosition(window);
                float radius = rand() % (particle_max_size-particle_min_size+1) + particle_min_size;
                float mass = PI*radius*radius;
                Vector2 position = Vector2{(float)mouse_position.x, (float)mouse_position.y};
                float dist = distance(position, boundary_center);
                if(dist > boundary_radius - radius) {
                    Vector2 norm = unit(boundary_center - position);
                    float overshot = (dist - boundary_radius + radius);
                    position = position + norm*overshot;
                }

                Particle *p = initialize(radius, mass, {position.x + rand() % 5, position.y + rand() % 5}, {0, 0});
                particles.push_back(p);
                spawn_speed_time = spawn_speed_time - spawn_speed;
            }
            spawn_speed_time += delta_time;
        }
        
        window.clear();

        window.draw(boundary);

        if(!pause_toggle) {
            // particle_to_partition(partition_size);
            energy = 0.f;
            sf::Vector2i position = sf::Mouse::getPosition(window);
            update(delta_time, rmb_toggle, {(float)position.x, (float)position.y}, gravity_toggle, &energy, &boundary_partitions, partition_size);   
        }

        draw_particles(&window);

        if(draw_toggle) {
            draw_velocity(&window);
            draw_partitions(partition_size, &boundary_partitions, &window, &font);
        }

        particle_count_text.setString("particle_count: " + to_string(particles.size()));
        window.draw(particle_count_text);

        kinetic_energy_text.setString("kinetic_energy: " + to_string((unsigned long int)energy) + " kJ");
        window.draw(kinetic_energy_text);

        fps = (int)(1/delta_time);
        fps_text.setString("fps: " + to_string(fps));
        window.draw(fps_text);

        fps = min(fps, 110);
        fps_bar.setSize(sf::Vector2f(fps, 4.f));

        window.draw(fps_bar);

        window.draw(controls_text);

        window.display();
    }

    return 0;
}