//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray);
    if(intersection.emit.norm() > 0){
        return intersection.emit;
    }
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = Vector3f(0,0,0);
    if(!intersection.happened){
        return this->backgroundColor;
    }
    Intersection inter_light;
    float pdf = 0;
    sampleLight(inter_light,pdf);
    Vector3f L_dir(0);
    Vector3f wi = normalize(intersection.coords - inter_light.coords);
    Vector3f N = normalize(intersection.normal);
    Vector3f wo = -normalize(ray.direction);
    Vector3f f_r = intersection.m->eval(wi,wo,N);
    float cos_theta = dotProduct(N,-wi);
    float cos_theta_x = dotProduct(inter_light.normal,wi);
    Vector3f dis = intersection.coords - inter_light.coords;
    Vector3f L_i = inter_light.emit;
    if((intersect(Ray(intersection.coords,-wi)).coords - inter_light.coords).norm() < 0.01){
        L_dir = wiseProduct(L_i,f_r) * cos_theta * cos_theta_x /pow(dis.norm(),2)/pdf;
    }
    Vector3f L_indir = Vector3f(0.0f);
    if(get_random_float() <= RussianRoulette){
        wi = -normalize(intersection.m->sample(-wo,N));
        pdf = intersection.m->pdf(wi,wo,N);
        f_r = intersection.m->eval(wi,wo,N);
        Ray r2 = Ray(intersection.coords,-wi);
        L_indir = castRay(r2,depth+1) * f_r * dotProduct(N,-wi)/pdf/RussianRoulette;
    }
    return L_dir + L_indir;
	// Intersection intersection = intersect(ray);
    // Vector3f hitcolor = Vector3f(0);

    // //deal with light source
    // if(intersection.emit.norm()>0)
    // hitcolor = Vector3f(1);
    // else if(intersection.happened)
    // {
    //     Vector3f wo = normalize(-ray.direction);
    //     Vector3f p = intersection.coords;
    //     Vector3f N = normalize(intersection.normal);

    //     float pdf_light = 0.0f;
    //     Intersection inter;
    //     sampleLight(inter,pdf_light);
    //     Vector3f x = inter.coords;
    //     Vector3f ws = normalize(x-p);
    //     Vector3f NN = normalize(inter.normal);

    //     Vector3f L_dir = Vector3f(0);
    //     //direct light
    //     if((intersect(Ray(p,ws)).coords - x).norm() < 0.01)
    //     {
    //         L_dir = inter.emit * intersection.m->eval(wo,ws,N)*dotProduct(ws,N) * dotProduct(-ws,NN) / (((x-p).norm()* (x-p).norm()) * pdf_light);
    //     }

    //     Vector3f L_indir = Vector3f(0);
    //     float P_RR = get_random_float();
    //     //indirect light
    //     if(P_RR < Scene::RussianRoulette)
    //     {
    //         Vector3f wi = intersection.m->sample(wo,N);
    //         L_indir = castRay(Ray(p,wi),depth) *intersection.m->eval(wi,wo,N) * dotProduct(wi,N) / (intersection.m->pdf(wi,wo,N)*Scene::RussianRoulette);
    //     }
    //     hitcolor = L_indir + L_dir;
    // }
    // return hitcolor;
}