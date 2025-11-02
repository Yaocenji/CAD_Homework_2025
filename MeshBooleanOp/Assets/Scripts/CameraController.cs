using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Camera))]
public class CameraController : MonoBehaviour
{
    private Camera cam;
    
    private bool isMoving = false;
    private Vector2 beginMousePos;
    private Vector2 beginCamPos;
    
    void Start()
    {
        cam = GetComponent<Camera>();
    }

    // Update is called once per frame
    void Update()
    {
        // 按下鼠标中键
        if (Input.GetMouseButtonDown(2))
        {
            beginMousePos =  Input.mousePosition;
            beginCamPos = cam.transform.position;
            isMoving = true;
            //Debug.Log("中键按下，当前鼠标位置：" + beginMousePos);
        }

        if (Input.GetMouseButtonUp(2))
        {
            isMoving = false;
            //Debug.Log("中键抬起");
        }

        if (isMoving)
        {
            Vector2 deltaMove = (Vector2)Input.mousePosition - beginMousePos;
            Vector2 newCamPos = beginCamPos - 2.0f * deltaMove / Screen.height * cam.orthographicSize;
            cam.transform.position = new Vector3(newCamPos.x, newCamPos.y, cam.transform.position.z);
        }

        if (Input.mouseScrollDelta.y != 0)
        {
            //Debug.Log(Input.mouseScrollDelta.y);
            //cam.orthographicSize += Input.mouseScrollDelta.y;
            if (Input.mouseScrollDelta.y > 0)
                cam.orthographicSize *= 0.85f;
            if (Input.mouseScrollDelta.y < 0)
                cam.orthographicSize *= 1.15f;
            
            cam.orthographicSize = Mathf.Clamp(cam.orthographicSize, 1.5f, 10.0f);
        }
    }
}
