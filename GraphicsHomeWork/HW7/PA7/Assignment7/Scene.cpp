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
    Vector3f hitColor(0,0,0);
    Intersection intersection =Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f N = intersection.normal;
    Vector3f p = intersection.coords;
    Vector3f wo = -ray.direction;
    if(intersection.happened)
    {
        // emit light 
        if(hitObject->hasEmit())
        {
            hitColor += m->getEmission();
        }

        // direct light
        float pdf_light = 0;
        Intersection light_insect;
        sampleLight(light_insect,pdf_light);
        auto emit = light_insect.emit ;
        auto x = light_insect.coords;
        auto NN = light_insect.normal;
        auto ws = normalize(p-x);
        Vector3f ray_dir = normalize(x-p);
        Ray direct_light(p,ray_dir);
        Intersection direct_insect = Scene::intersect(direct_light);
        Vector3f light_dir(0,0,0);
        if(direct_insect.happened && direct_insect.obj->hasEmit())
        {
            light_dir = emit*m->eval(wo,ray_dir,N)*dotProduct(ray_dir,N)*dotProduct(ws,NN)/pow((x-p).norm(),2)/pdf_light;
        } 
        hitColor += light_dir;
        
        // indirect light
        Vector3f light_indir(0,0,0);
        if(get_random_float()<=RussianRoulette)
        {
            Vector3f wi = m->sample(wo,N);
            Ray indir_light(p,wi);
            Intersection object_inset = Scene::intersect(indir_light);
            float min_pdf = 1e-8;
            if(object_inset.happened && !object_inset.obj->hasEmit())
            {
                light_indir = castRay(indir_light,depth+1)*m->eval(wo,wi,N)*dotProduct(wi,N)/std::max(min_pdf,m->pdf(wo,wi,N))/RussianRoulette;
            }
        }
        hitColor += light_indir;
    }
    return hitColor;
}