# Notes

## Theory
1. Filter image by color
2. Find edges with Canny Edge Detector
3. Crop irrelevant parts of the images
4. Detect lines with Hough Transform

## Filter Image by Color
1. I need to detect only a white line. So i decided to use grayscale filter
2. Next, i pcked a threshold to have a binary image

## Detect Line
1. Use Canny Edge detector to detect line
2. Use Hough line detector

Alternatively, if can't get satisfying result. Can try below method

## Contour Detection
1. Assuming the biggest contour is the line and taking centerline of the bounding box. I got plausible direction


## To summarize
1. convert BGR image into Gray. Gray image contain value ranging from [0-255]
```
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

```
2. image smoothing
```
# given input image, kernel width =5 height = 5, Gaussian kernel standard deviation
k_width = 5
k_height = 5
blurred = cv2.GaussianBlur(gray, (k_width, k_height), 0)
```

3. Edge detection
```
# Find the edges in the image using canny detector
threshold1 = 80
threshold2 = 80
edged = cv2.Canny(blurred, threshold1, threshold2)
```

4. Find lines
```
# it will return line coordinates,it will return 3darray.
lines = cv2.HoughLinesP(edged,1,np.pi/180,max_slider,minLineLength,maxLineGap)

for x in range(0, len(lines)):
    for x1,y1,x2,y2 in lines[x]:
        # draw line in image using cv2.line function.
        cv2.line(image,(x1,y1),(x2,y2),(255,0,0),3)
        theta=theta+math.atan2((y2-y1),(x2-x1))
        print(theta)

you can take the coordinate of first line by using this line of code.
```

5. Decision Making Part
```
threshold=5
if(theta>threshold):
    print("Go left")
if(theta<-threshold):
    print("Go right")
if(abs(theta)<threshold):
    print("Go straight")
```

6. Integrate
```
// assign pin num
int right_pin = 12;
int left_pin = 13;
int const ENA = 10;  
int const ENB = 11; 


// initial command
int command = 0;

void setup()

{
  pinMode(right_pin, OUTPUT);
  pinMode(left_pin, OUTPUT);
  pinMode(ENA, OUTPUT);   // set all the motor control pins to outputs
  pinMode(ENB, OUTPUT);

  Serial.begin(115200);
  
  while (!Serial);
  
  Serial.println("Opencv Lane Detect Autonomous Robot");

}

void loop() {

if (Serial.available())

{
  //int state = Serial.parseInt();
  int state = Serial.read();

  if (state == 4)
  {
    
    digitalWrite(left_pin, LOW);
    digitalWrite(right_pin, HIGH);
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    Serial.println("Left");
  }

  if (state == 2)
  {
    digitalWrite(left_pin, LOW);
    digitalWrite(right_pin, LOW);
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);
    Serial.println("Right");
    Serial.println("Reverse");
  }

  if (state == 3)
  {
    digitalWrite(left_pin, HIGH);
    digitalWrite(right_pin, LOW);
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);

  Serial.println("Right");
  }
  if (state == 1)
  {
    digitalWrite(left_pin, HIGH);
    digitalWrite(right_pin, HIGH);
    digitalWrite(ENA, HIGH);
    digitalWrite(ENB, HIGH);

  Serial.println("Forward");
  }
  if (state == 5)
  {
    digitalWrite(left_pin, LOW);
    digitalWrite(right_pin, LOW);
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);

  Serial.println("Stop");
  }
}
}
```