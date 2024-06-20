#include "planar_quadrotor_visualizer.h"


PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */

SDL_Point rotate_point(double cx, double cy, double angle, SDL_Point p){
    double pi = acos(-1);
    double rotation_angle = (double)angle / 180.0 * pi;
    double s = sin(rotation_angle);
    double c = cos(rotation_angle);

    // translate point back to origin:
    p.x -= cx;
    p.y -= cy;

    // rotate point
    double xnew = p.x * c - p.y * s;
    double ynew = p.x * s + p.y * c;

    // translate point back:
    p.x = xnew + cx;
    p.y = ynew + cy;

    return p;
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    Eigen::VectorXf goal = quadrotor_ptr->Getgoal();

    float g_x,g_y;

    g_x = goal[0];
    g_y = goal[1];


    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];
    

    //center rectangle points
    SDL_Point point1;
    point1.x = q_x - 15;
    point1.y = q_y - 10;

    SDL_Point point2;
    point2.x = q_x + 15;
    point2.y = q_y - 10;
    
    SDL_Point point3;
    point3.x = q_x + 15;
    point3.y = q_y + 10;
    
    SDL_Point point4;
    point4.x = q_x - 15;
    point4.y = q_y + 10;


    //rect 1 points
    SDL_Point point11;
    point11.x = q_x - 15;
    point11.y = q_y - 5;

    SDL_Point point12;
    point12.x = q_x - 50;
    point12.y = q_y - 5;

    SDL_Point point12l;
    point12l.x = q_x - 50;
    point12l.y = q_y - 15;
    
    SDL_Point point13;
    point13.x = q_x - 50;
    point13.y = q_y + 5;

    SDL_Point point14;
    point14.x = q_x - 15;
    point14.y = q_y + 5;

    //rect 2 points
    SDL_Point point21;
    point21.x = q_x + 15;
    point21.y = q_y - 5;

    SDL_Point point22;
    point22.x = q_x + 50;
    point22.y = q_y - 5;

    SDL_Point point22p;
    point22p.x = q_x + 50;
    point22p.y = q_y - 15;

    SDL_Point point23;
    point23.x = q_x + 50;
    point23.y = q_y + 5;

    SDL_Point point24;
    point24.x = q_x + 15;
    point24.y = q_y + 5;

    //proppeler
    SDL_Point point31l;
    point31l.x = q_x - 75;
    point31l.y = point12l.y;

    SDL_Point point32l;
    point32l.x = q_x - 25;
    point32l.y = point12l.y;

    SDL_Point point31p;
    point31p.x = q_x + 75;
    point31p.y = point12l.y;

    SDL_Point point32p;
    point32p.x = q_x + 25;
    point32p.y = point12l.y;

    //proppeller actuation
    static float b = 0.0f;
    static bool a = true;
    if(b >= 50.0f)
        a = false;
    if(b <= 0.0f)
        a = true;

    if(a){
        b += 0.5f;
    }
    else
        b -= 0.5f;

        point31l.x += b;
        point32l.x -= b;
        point31p.x -= b;
        point32p.x += b;

    //rotate points around center
    point1 = rotate_point(q_x, q_y, q_theta, point1);
    point2 = rotate_point(q_x, q_y, q_theta, point2);
    point3 = rotate_point(q_x, q_y, q_theta, point3);
    point4 = rotate_point(q_x, q_y, q_theta, point4);

    point11 = rotate_point(q_x, q_y, q_theta, point11);
    point12 = rotate_point(q_x, q_y, q_theta, point12);
    point12l = rotate_point(q_x, q_y, q_theta, point12l);
    point13 = rotate_point(q_x, q_y, q_theta, point13);
    point14 = rotate_point(q_x, q_y, q_theta, point14);

    point21 = rotate_point(q_x, q_y, q_theta, point21);
    point22 = rotate_point(q_x, q_y, q_theta, point22);
    point22p = rotate_point(q_x, q_y, q_theta, point22p);
    point23 = rotate_point(q_x, q_y, q_theta, point23);
    point24 = rotate_point(q_x, q_y, q_theta, point24);

    point31l = rotate_point(q_x, q_y, q_theta, point31l);
    point32l = rotate_point(q_x, q_y, q_theta, point32l);
    point31p = rotate_point(q_x, q_y, q_theta, point31p);
    point32p = rotate_point(q_x, q_y, q_theta, point32p);

    SDL_Point points[5]  = {point1, point2, point3, point4, point1};
    SDL_Point points1[6] = {point11, point12, point12l, point13, point14};
    SDL_Point points2[6] = {point21, point22, point22p, point23, point24};
    SDL_Point points3l[2] = {point31l, point12l};
    SDL_Point points4l[2] = {point12l, point32l};
    SDL_Point points3p[2] = {point31p, point22p};
    SDL_Point points4p[2] = {point22p, point32p};

    //render drone
    SDL_SetRenderDrawColor(gRenderer.get(), 0, 0, 0, 0);            //black
    SDL_RenderDrawLines(gRenderer.get(), points, 5);
    SDL_RenderDrawLines(gRenderer.get(), points1, 5);
    SDL_RenderDrawLines(gRenderer.get(), points2, 5);

    //render red propeller arms
    SDL_SetRenderDrawColor(gRenderer.get(), 255, 0, 0, 255);        //red
    SDL_RenderDrawLines(gRenderer.get(), points3l, 2);
    SDL_RenderDrawLines(gRenderer.get(), points3p, 2);

    //render blue propeller arms
    SDL_SetRenderDrawColor(gRenderer.get(), 0, 0, 255, 255);        //blue
    SDL_RenderDrawLines(gRenderer.get(), points4l, 2);
    SDL_RenderDrawLines(gRenderer.get(), points4p, 2);

    //goal dot
    filledCircleColor(gRenderer.get(), g_x, g_y, 5, 0xFFFF11FF);

}