using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class DrawMesh : MonoBehaviour
{
    public Material material1;
    public Material material2;
    
    private List<LineRenderer> _lineRenderers1;
    
    void Start()
    {
        _lineRenderers1 =  new List<LineRenderer>();
        
        //MeshManager.Instance.meshA.LineRenderers =  _lineRenderers1;
    }

    // Update is called once per frame
    void Update()
    {
        if (MeshManager.Instance.meshA is not null && MeshManager.Instance.meshA.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshA, material1);
        }
        if (MeshManager.Instance.meshB is not null && MeshManager.Instance.meshB.needUpdate)
        {
            UpdateMeshAndLineRenderers(MeshManager.Instance.meshB, material2);
        }
    }

    private void UpdateMeshAndLineRenderers(Mesh mesh , Material material)
    {
        
        mesh.needUpdate = false;
        if (mesh.LineRenderers is not null)
        {
            foreach (LineRenderer lr in mesh.LineRenderers)
            {
                // 删去所有的lineRenderer，并删去挂接它的物体
                GameObject go = lr.gameObject;
                Destroy(lr);
                Destroy(go);
            }
            mesh.LineRenderers.Clear();
            
            for (int i = 0; i < mesh.Loops.Count; i++)
            {
                // 创建mesh的子物体，并挂接lineRenderer
                GameObject go = new GameObject();
                go.transform.parent = mesh.transform;
                LineRenderer newLineRenderer = go.AddComponent<LineRenderer>();
                
                // 修改lineRenderer
                newLineRenderer.material = material;
                
                newLineRenderer.startWidth = 0.1f;
                newLineRenderer.endWidth = 0.1f;
                newLineRenderer.loop = true;
                
                var positions = mesh.Loops[i].GetPositions();
                newLineRenderer.positionCount = positions.Length;
                newLineRenderer.SetPositions(positions);
                
                mesh.LineRenderers.Add(newLineRenderer);
            }
        }
    }
}
