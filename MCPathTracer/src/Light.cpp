#include "Light.h"
#include "Model.h"


void AreaLight::AddMesh(Mesh* m)
{
    pMesh = m;
    for (int i = 0; i < (pMesh->TriList.size()); ++i)
    {
        area += pMesh->TriList[i]->getArea();
    }
}