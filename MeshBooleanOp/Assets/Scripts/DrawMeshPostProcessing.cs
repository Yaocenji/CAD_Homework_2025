using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Camera))]
public class DrawMeshPostProcessing : MonoBehaviour
{
    public Shader drawMeshShader;
    
    private Material drawMeshMaterial;

    private void Start()
    {
        drawMeshMaterial = new Material(drawMeshShader);
    }

    private void OnRenderImage(RenderTexture source, RenderTexture destination)
    {
        Graphics.Blit(source, destination, drawMeshMaterial);
    }
}
