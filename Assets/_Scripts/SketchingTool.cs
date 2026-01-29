using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

public enum DeletionFrame
{
	NULL = 0,
	StraightenDrawing = 1,
	Drawing = 2,
}

public class SketchingTool : MonoBehaviour
{
	[Header("References")]
	[SerializeField] private SketchToMesh sketchToMesh;
	[SerializeField] private RawImage rawImage;

	#region Drawing
	[Header("Drawing Settings")]
	[SerializeField] private Color drawColor = Color.black;
	[Range(2, 5)]
	[SerializeField] private int brushSize = 5;
	public int BrushSize => brushSize;
	[SerializeField] private Transform sketchCollector;

	public Texture2D texture { private set; get; }
	private Texture2D previewTexture;

	private bool isDrawing = false;
	private bool isStraightening = false;
	private DeletionFrame deletedLastFrame = DeletionFrame.NULL;

	private Vector2? firstDrawnPos = null;
	private Vector2? previousDrawPos = null;
	private Vector2? straightenStartPos = null;
	#endregion

	#region Camera Controls
	[Header("Input Settings")]
	[SerializeField] private InputActionAsset inputActions;
	private string actionMapName = "Player";

	private Camera mainCamera;
	private float cameraPitch = 0f;

	private InputAction moveAction; 
	private InputAction lookAction;
	private InputAction verticalMoveAction;
	private InputAction deleteAction;
	#endregion

	#region Public Functions
	public Texture2D GetFinalTexture()
	{
		return texture;
	}
	#endregion

	#region Start Settings
	private void Start()
	{
		ApplyTextures();
		SetUpControls();
	}

	private void ApplyTextures()
	{
		texture = new Texture2D(1920, 1080, TextureFormat.RGBA32, false);
		previewTexture = new Texture2D(1920, 1080, TextureFormat.RGBA32, false);
		rawImage.color = new Color(rawImage.color.r, rawImage.color.g, rawImage.color.b, 1);

		Color transparent = new Color(1, 1, 1, 0);
		Color[] fillPixels = new Color[texture.width * texture.height];
		for (int i = 0; i < fillPixels.Length; i++)
			fillPixels[i] = transparent;

		texture.SetPixels(fillPixels);
		texture.Apply();

		previewTexture.SetPixels(fillPixels);
		previewTexture.Apply();

		rawImage.texture = texture;
	}

	private void SetUpControls()
	{
		mainCamera = Camera.main;

		var map = inputActions.FindActionMap(actionMapName);
		moveAction = map.FindAction("Move");
		lookAction = map.FindAction("Look");
		verticalMoveAction = map.FindAction("VerticalMove");
		deleteAction = map.FindAction("Delete");

		moveAction.Enable();
		lookAction.Enable();
		verticalMoveAction.Enable();
		deleteAction.Enable();
	}
	#endregion

	private void Update()
	{
		bool isRightMousePressed = Mouse.current.rightButton.wasPressedThisFrame;
		bool isRightMouseHeld = Mouse.current.rightButton.isPressed;
		bool isLeftMousePressed = Mouse.current.leftButton.wasPressedThisFrame;
		bool isLeftMouseHeld = Mouse.current.leftButton.isPressed;
		bool isLeftMouseReleased = Mouse.current.leftButton.wasReleasedThisFrame;

		if (deleteAction.WasPressedThisFrame())
		{
			if (isDrawing)
			{
				if (isStraightening) deletedLastFrame = DeletionFrame.StraightenDrawing;
				else deletedLastFrame = DeletionFrame.Drawing;
			}
			else
			{
				int lastIndex = sketchCollector.childCount - 1;
				Destroy(sketchCollector.GetChild(lastIndex).gameObject);
			}

			ClearCanvas();
			return;
		}
		else if (deletedLastFrame != DeletionFrame.NULL)
		{
			if (deletedLastFrame == DeletionFrame.StraightenDrawing)
			{
				isStraightening = true;
				straightenStartPos = GetCurrentTexturePosition();
			}

			isDrawing = true;
			firstDrawnPos = GetCurrentTexturePosition();

			deletedLastFrame = DeletionFrame.NULL;
		}

		if (isRightMouseHeld && !isDrawing && !isStraightening)
		{
			CameraControl();
			return;
		}

		if (Keyboard.current.spaceKey.wasPressedThisFrame)
		{
			isStraightening = true;
			straightenStartPos = GetCurrentTexturePosition();
		}

		if (Keyboard.current.spaceKey.wasReleasedThisFrame)
		{
			if (isStraightening)
			{
				CommitPreviewToTexture();
				ClearTexture(previewTexture);
				previewTexture.Apply();
				MergeTextures();

				isStraightening = false;
				straightenStartPos = null;
				previousDrawPos = GetCurrentTexturePosition();
			}
		}

		if (isStraightening && isRightMousePressed)
		{
			CommitPreviewToTexture();
			straightenStartPos = GetCurrentTexturePosition();
		}

		if (isLeftMousePressed)
		{
			ClearCanvas();
			isDrawing = true;

			if (firstDrawnPos == null) firstDrawnPos = GetCurrentTexturePosition();
			if (isStraightening)
			{
				CommitPreviewToTexture();
				straightenStartPos = GetCurrentTexturePosition();
			}
			else previousDrawPos = GetCurrentTexturePosition();
		}

		if (isLeftMouseReleased)
		{
			if (isDrawing)
			{
				if (isStraightening)
				{
					CommitPreviewToTexture();

					previousDrawPos = GetCurrentTexturePosition();
					straightenStartPos = null;

					ClearTexture(previewTexture);
					isStraightening = false;
				}
				if (firstDrawnPos != null && previousDrawPos != null)
				{
					DrawLine(texture, (Vector2)previousDrawPos, (Vector2)firstDrawnPos);
					texture.Apply();
				}

				isDrawing = false;

				sketchToMesh.GenerateMeshFromSketch();
				//ClearCanvas();
			}

			previousDrawPos = null;
			firstDrawnPos = null;
		}

		if (isDrawing && (isLeftMouseHeld || isStraightening))
		{
			Vector2 currentDrawPos = GetCurrentTexturePosition();

			if (isStraightening)
			{
				UpdatePreviewLine(straightenStartPos, currentDrawPos);
			}
			else
			{
				if (previousDrawPos != null)
				{
					DrawLine(texture, (Vector2)previousDrawPos, currentDrawPos);
				}
				else
				{
					DrawCircle(texture, (int)currentDrawPos.x, (int)currentDrawPos.y);
				}
				previousDrawPos = currentDrawPos;
				texture.Apply();
			}

			MergeTextures();
		}
		else
		{
			MergeTextures();
		}
	}

	private void ClearCanvas()
	{
		isStraightening = false;
		isDrawing = false;
		straightenStartPos = null;
		previousDrawPos = null;
		firstDrawnPos = null;

		ClearTexture(previewTexture);
		previewTexture.Apply();
		ClearTexture(texture);
		texture.Apply();
		MergeTextures();
	}

    private void CameraControl()
    {
        Vector2 moveInput = moveAction.ReadValue<Vector2>();
        Vector2 lookInput = lookAction.ReadValue<Vector2>();
        float verticalInput = verticalMoveAction.ReadValue<float>();

        float speed = 10f;
        float rotateSpeed = 1f;

        Vector3 move = new Vector3(moveInput.x, 0, moveInput.y);
        Vector3 verticalMove = new Vector3(0, verticalInput, 0);
        mainCamera.transform.Translate((move + verticalMove) * speed * Time.deltaTime, Space.Self);

        mainCamera.transform.Rotate(Vector3.up, lookInput.x * rotateSpeed, Space.World);

        cameraPitch -= lookInput.y * rotateSpeed;
        cameraPitch = Mathf.Clamp(cameraPitch, -89f, 89f);
        Vector3 currentEuler = mainCamera.transform.eulerAngles;
        currentEuler.x = cameraPitch;
        mainCamera.transform.eulerAngles = currentEuler;
    }

	#region Drawing Helpers
	private Vector2 GetCurrentTexturePosition()
	{
		Vector2 localPos;
		RectTransformUtility.ScreenPointToLocalPointInRectangle(
			rawImage.rectTransform,
			Mouse.current.position.ReadValue(),
			null,
			out localPos
		);

		Rect rect = rawImage.rectTransform.rect;
		float normX = Mathf.InverseLerp(rect.xMin, rect.xMax, localPos.x);
		float normY = Mathf.InverseLerp(rect.yMin, rect.yMax, localPos.y);

		int texX = Mathf.RoundToInt(normX * texture.width);
		int texY = Mathf.RoundToInt(normY * texture.height);

		return new Vector2(texX, texY);
	}

	private void DrawCircle(Texture2D tex, int centerX, int centerY)
	{
		for (int y = -brushSize; y <= brushSize; y++)
		{
			for (int x = -brushSize; x <= brushSize; x++)
			{
				if (x * x + y * y <= brushSize * brushSize)
				{
					int px = centerX + x;
					int py = centerY + y;
					if (px >= 0 && px < tex.width && py >= 0 && py < tex.height)
					{
						Color drawPixel = new Color(drawColor.r, drawColor.g, drawColor.b, 1f);
						tex.SetPixel(px, py, drawPixel);
					}
				}
			}
		}
	}

	private void DrawLine(Texture2D tex, Vector2? start, Vector2 end)
	{
		Vector2 s = (Vector2)start;

		int points = (int)Vector2.Distance(s, end);
		for (int i = 0; i <= points; i++)
		{
			float t = (float)i / points;
			int x = Mathf.RoundToInt(Mathf.Lerp(s.x, end.x, t));
			int y = Mathf.RoundToInt(Mathf.Lerp(s.y, end.y, t));
			DrawCircle(tex, x, y);
		}
	}

	private void UpdatePreviewLine(Vector2? start, Vector2 end)
	{
		if (start == null) return;

		ClearTexture(previewTexture);
		DrawLine(previewTexture, start, end);
		previewTexture.Apply();
	}

	private void CommitPreviewToTexture()
	{
		Color[] previewPixels = previewTexture.GetPixels();
		Color[] mainPixels = texture.GetPixels();

		for (int i = 0; i < mainPixels.Length; i++)
		{
			if (previewPixels[i].a > 0f)
			{
				mainPixels[i] = previewPixels[i];
			}
		}

		texture.SetPixels(mainPixels);
		texture.Apply();
	}

	private void ClearTexture(Texture2D tex)
	{
		Color[] clearPixels = new Color[tex.width * tex.height];
		for (int i = 0; i < clearPixels.Length; i++)
			clearPixels[i] = new Color(1, 1, 1, 0);

		tex.SetPixels(clearPixels);
		tex.Apply();
	}

	private void MergeTextures()
	{
		Texture2D combined = new Texture2D(texture.width, texture.height, TextureFormat.RGBA32, false);

		Color[] mainPixels = texture.GetPixels();
		Color[] previewPixels = previewTexture.GetPixels();
		Color[] combinedPixels = new Color[mainPixels.Length];

		for (int i = 0; i < mainPixels.Length; i++)
		{
			combinedPixels[i] = mainPixels[i];

			if (previewPixels[i].a > 0f)
			{
				combinedPixels[i] = previewPixels[i];
			}
		}

		combined.SetPixels(combinedPixels);
		combined.Apply();

		rawImage.texture = combined;
	}
	#endregion
}