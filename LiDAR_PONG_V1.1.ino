// LiDAR_PONG_V1.X

// Arduino code for LiDAR PONG EPE demo
// Author(s): Jordan McCaughey Walsh
// > Contact: jmcc097@gmail.com 



// Intendid for use with ARM based Arduino boards (Arduino DUE)
// Matrix size 24x32




//V1.0 [Jordan] (04/08/23):
//   Code used for inital Presentation on 10/08/23.

//V1.0.1 [Jordan] (19/08/23):
//  - Added collision (bug fix) and 180 degree rotortion for collisopns on tip of paddles (adds additional skill element to game).
//  - Fixed game soft-lock bug in case where measurement successfully read on first attempt in TFminiGetDistance()
//     > This did not actually fix the issue, but is still a bug. The issue could be related to the current physical setup where the
//       Arduino is driving the current to LED matrix rather than an external power supply)
//  - Added 'easyMode' toggle for children - This should be toggled with a physical switch ideally.

//V1.0.2 [Jordan] (11/23)
//  - Adjusted Ball spawning parameters in reset(), previous values were not competitively balanced. 
//     > initial ball direction should now be less predictable.
//     > more fairly confined spawn region.

//V1.1 [Jordan] (03/07/24)
//  - Fixed soft-lock issue (finally). 
//     > 'right' and 'left' paddle positions changed from 'unsigned int' to 'int'. 'right' would go negative
//       after sensor read 24, correspnding to 0 in game space but because of unsigned data type NeoMatrix received 4294967295.
//  - Adjusted duration of end screen
//  - Added timer/decontruction of walls during endscreen
//  - Added more angles for the ball to traverse + suitable collision detection 
//     > 202, 257, 337, 22



//V1.2 [Jordan]
//  - Feature reccomendations:
//     > even more angles for the ball to traverse + suitable collision detection.
//     > a more physics based collision detection that takes into account the velocity of the paddle and direction of the ball.
//     > skins + stages / backgrounds?
//     > IPIC/Tyndall logo might be nice.
//     > Ability to change colour of paddles.




#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#ifndef PSTR
 #define PSTR // Make Arduino Due happy
#endif
#define PIN 22
#define DEFAULT_SPEED 3; //starting speed (RECOMMENDED VALUE 3)
#define LIDAR_DIVIDE 1; //

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 32, 3, 1, PIN,
  NEO_TILE_TOP   + NEO_TILE_LEFT   + NEO_TILE_ROWS   + NEO_TILE_ZIGZAG +
  NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
  NEO_GRB + NEO_KHZ800);

int left = 0;  // V1.1: DO NOT UNSIGN!
int right = 0; // V1.1: DO NOT UNSIGN!

int angle = 0;
int radians;
int _angle;
int _px = 16; //boot up initial conditions for ball
int _py = 12; // "
int _w = 26;
int _h = 22;
int _wall[2];
int _count = 0;
int _speed = DEFAULT_SPEED;
int _countPoints = 0;
int colision = 0;
int LEDbrightness;
bool gameOver;
bool easyMode;
int winner;
int player1_score = 0;
int player2_score = 0;
int y_1 = 5; //paddle y positions
int y_2 = 26; // ""

void getTFminiData(HardwareSerial &target, int* distance, int* strength){
  //Serial.print("getTFminiData\n");
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (target.available()){
    rx[i] = target.read();
    if (rx[0] != 0x59){
      i = 0;
    }
    else if (i == 1 && rx[1] != 0x59){
      i = 0;
    }
    else if (i == 8){
      for (j = 0; j < 8; j++){
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)){
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    }
    else{
      i++;
    }
  }
}

int getTFminiDistance(HardwareSerial &target){ // return distance read for a target sensor as an integer - this is rapid
  //Serial.print("getTFminiDistance\n");
  int distance = 0;
  int strength = 0;
  getTFminiData(target, &distance, &strength);
  while (!distance){
    getTFminiData(target, &distance, &strength);
    if (distance){
      return distance;
    }
  }
  return 1;
  // ^ V1.1 EDIT
}

// PONG logic and fucntions \\

void calcWall() {
  //Serial.print("calcWall\n");
  //padel placements and detection
  left = getTFminiDistance(Serial1)*LIDAR_DIVIDE; // player1
  //Serial.print("calcWall1\n");
  right = 24-getTFminiDistance(Serial2)*LIDAR_DIVIDE; //player2
  //right = getTFminiDistance(Serial2)*LIDAR_DIVIDE; //player2
  //Serial.println(right);

  //Draw Padels
  for(int x = left-2; x <= left+2; x++){
    matrix.drawPixel(x, y_1, matrix.Color(0, 255, 0));
  }
  //Serial.print("left_drawn\n");
  for(int x = right-2; x <= right+2; x++){ 
    matrix.drawPixel(x, y_2, matrix.Color(255, 255, 255));
  }
  //Serial.print("right_drawn\n");
  _wall[0] = left;  //bottom
  _wall[1] = right; //top
  //Serial.print("RETURNING\n");
}

void enterFrameHandler(){
  //Serial.print("enterFrameHandler\n");
  _count++;
  if (_count < _speed) {
    matrix.drawPixel(_py, _px, matrix.Color(255, 255, 255));
    matrix.show();
  } else {
    _count = 0;
    checkCollision();
    calcAngleIncrement();
    matrix.drawPixel(_py, _px, matrix.Color(255, 255, 255));
    matrix.show();
  }
}

void retorted(int angle) { 
  _angle = angle;
  if (easyMode == false)  colision++;
}

void reset() { //resets animation after a player fails to deflect the ball and determines angle of ball bouncing
  //ballPop();
  bool go = false;
  int count = 0;
  _px = 15;//random(16, 18);//EDIT V1.1
  _py = 12;//random(6, 26); //EDIT V1.1
  _angle = random(0, 2) < 1 ? 0 : 180; //EDIT V1.1
  _speed = DEFAULT_SPEED;
  colision = 0;
  _countPoints = 0;
  while(go == false){
    matrix.clear();
    count++;
    drawWalls();
    test();
    //Ball Flashes in spawn position
    if(count%2 == 0){
      matrix.drawPixel(_py, _px, matrix.Color(255, 255, 255));
    }
    else {
      matrix.drawPixel(_py, _px, matrix.Color(0, 0, 0));
    }
    if(count%20 == 0){
      go = true;
      count = 0;
    }
    matrix.show();
  }
}

void checkCollision() {
  //_px or _py is ball position coordinates
  if(_px == _w - 1){ //paddel position - 1 // HIGH side
    //buzz();
    if (_angle == 315 || _angle == 0 || _angle == 45 || _angle == 22 || _angle == 337)  {
      if (_py == _wall[1] || _py == _wall[1]+1 || _py == _wall[1]+2 || _py == _wall[1]-1 || _py == _wall[1]-2 || _py == _wall[1]-3 || _py == _wall[1]+3)  {
        if (_angle == 0 && _py == _wall[1])  retorted(180);
        else if (_angle == 0 && _py == _wall[1] + 1)  retorted(157);
        else if (_angle == 0 && _py == _wall[1] + 2)  retorted(135);
        else if (_angle == 0 && _py == _wall[1] - 1)  retorted(202); 
        else if (_angle == 0 && _py == _wall[1] - 2)  retorted(225); 

        else if (_angle == 45 && _py == _wall[1])  retorted(135);
        else if (_angle == 45 && _py == _wall[1] + 1)  retorted(135);
        else if (_angle == 45 && _py == _wall[1] + 2)  retorted(135); 
        else if (_angle == 45 && _py == _wall[1] - 1)  retorted(157); 
        else if (_angle == 45 && _py == _wall[1] - 2)  retorted(180); 

        else if (_angle == 315 && _py == _wall[1])  retorted(225); 
        else if (_angle == 315 && _py == _wall[1] + 1)  retorted(157); 
        else if (_angle == 315 && _py == _wall[1] + 2)  retorted(180); 
        else if (_angle == 315 && _py == _wall[1] - 1)  retorted(225); 
        else if (_angle == 315 && _py == _wall[1] - 2)  retorted(225); 

        //V1.2 NEW ANGLES
        else if (_angle == 337 && _py == _wall[1])  retorted(202); 
        else if (_angle == 337 && _py == _wall[1] + 1)  retorted(180); 
        else if (_angle == 337 && _py == _wall[1] + 2)  retorted(157); 
        else if (_angle == 337 && _py == _wall[1] - 1)  retorted(225); 
        else if (_angle == 337 && _py == _wall[1] - 2)  retorted(225); 

        else if (_angle == 22 && _py == _wall[1])  retorted(157);
        else if (_angle == 22 && _py == _wall[1] + 1)  retorted(135);
        else if (_angle == 22 && _py == _wall[1] + 2)  retorted(135); 
        else if (_angle == 22 && _py == _wall[1] - 1)  retorted(180); 
        else if (_angle == 22 && _py == _wall[1] - 2)  retorted(202);         

        //V1.1 EDIT
        else if (_angle == 315 && _py == _wall[1] + 3)  retorted(135);
        else if (_angle == 337 && _py == _wall[1] + 3)  retorted(157);
        else if (_angle == 45 && _py == _wall[1] - 3)  retorted(225);
        else if (_angle == 22 && _py == _wall[1] - 3)  retorted(202);
      }
    }
  }
  else if (_px == 6)  { //paddel position + 1 //low side
    //buzz();
    if (_angle == 225 || _angle == 180 || _angle == 135 || _angle == 157 || _angle == 202)  {
      if (_py == _wall[0] || _py == _wall[0]+1 || _py == _wall[0]+2 || _py == _wall[0]-1 || _py == _wall[0]-2 || _py == _wall[0]+3 || _py == _wall[0]-3)  {
        if (_angle == 180 && _py == _wall[0])  retorted(0);
        else if (_angle == 180 && _py == _wall[0] + 1)  retorted(22);
        else if (_angle == 180 && _py == _wall[0] + 2)  retorted(45);
        else if (_angle == 180 && _py == _wall[0] - 1)  retorted(337); 
        else if (_angle == 180 && _py == _wall[0] - 2)  retorted(315);

        else if (_angle == 135 && _py == _wall[0])  retorted(45);
        else if (_angle == 135 && _py == _wall[0] + 1)  retorted(22); //SWAP?
        else if (_angle == 135 && _py == _wall[0] + 2)  retorted(45);   
        else if (_angle == 135 && _py == _wall[0] - 1)  retorted(337); //SWAP?
        else if (_angle == 135 && _py == _wall[0] - 2)  retorted(0);

        else if (_angle == 225 && _py == _wall[0])  retorted(315); 
        else if (_angle == 225 && _py == _wall[0] + 1)  retorted(337); 
        else if (_angle == 225 && _py == _wall[0] + 2)  retorted(0); 
        else if (_angle == 225 && _py == _wall[0] - 1)  retorted(315); 
        else if (_angle == 225 && _py == _wall[0] - 2)  retorted(315); 

        //V1.2 NEW ANGLES
        else if (_angle == 202 && _py == _wall[0])  retorted(337); 
        else if (_angle == 202 && _py == _wall[0] + 1)  retorted(0); 
        else if (_angle == 202 && _py == _wall[0] + 2)  retorted(0); 
        else if (_angle == 202 && _py == _wall[0] - 1)  retorted(315); 
        else if (_angle == 202 && _py == _wall[0] - 2)  retorted(315); 

        else if (_angle == 157 && _py == _wall[0])  retorted(22);
        else if (_angle == 157 && _py == _wall[0] + 1)  retorted(45); //SWAP?
        else if (_angle == 157 && _py == _wall[0] + 2)  retorted(45);   
        else if (_angle == 157 && _py == _wall[0] - 1)  retorted(0); //SWAP?
        else if (_angle == 157 && _py == _wall[0] - 2)  retorted(337);

        //V1.1 EDIT
        else if (_angle == 135 && _py == _wall[0] - 3)  retorted(315);
        else if (_angle == 225 && _py == _wall[0] + 3)  retorted(45);
        else if (_angle == 157 && _py == _wall[0] - 3)  retorted(315);
        else if (_angle == 202 && _py == _wall[0] + 3)  retorted(45);

      }
    }
  }
  if(colision == 2 && _speed > -1){
    colision = 0;
    _speed--;
  }
  if (_px == _w+2)  {
    player1_score++;
    reset();
  }
  else if (_px == 3)  {
    player2_score++;
    reset();
  }
  else if (_py == _h)  { //these are the side walls
    if (_angle == 45)  _angle = 315;
    else if (_angle == 135)  _angle = 225;
    else if (_angle == 157) _angle = 202;
    else if (_angle == 22) _angle = 337;
  }
  else if (_py == 1)  {
    if (_angle == 225)  _angle = 135;
    else if (_angle == 315)  _angle = 45;
    else if (_angle == 202) _angle = 157;
    else if (_angle == 337) _angle = 22;
  }
}

void calcAngleIncrement() {
  if (_angle == 0 || _angle == 360)  {
    _px += 1;
  }
  else if (_angle == 45)  {
    _px += 1;
    _py += 1;
  }
  else if (_angle == 135)  {
    _px -= 1;
    _py += 1;
  }
  else if (_angle == 180)  {
    _px -= 1;
  }
  else if (_angle == 225)  {
    _px -= 1;
    _py -= 1;
  }
  else if (_angle == 315)  {
    _px += 1;
    _py -= 1;
  }

  //New Angles V1.2 - 'if x position divisible by 2 then traverse in y'
  else if (_angle == 337)  {
    if (_px%2==0) _py -= 1;
    _px += 1;
    
    
    //_py -= 1;
  }
  else if (_angle == 22)  {
    if (_px%2==0) _py += 1;
    _px += 1;
    
    
    //_py -= 1;
  }
  else if (_angle == 157)  {
    if (_px%2==0) _py += 1;
    _px -= 1;
    
    
    //_py -= 1;
  }
  else if (_angle == 202)  {
    if (_px%2==0) _py -= 1;
    _px -= 1;
    //if (_px%2==0) _py -= 1;
    
    //_py -= 1;
  }                  // MIGHT WANT TO SWAP ORDER OF _px or _py in above statements!
}

// Primary Game Operation //

void menu(){ //logic for starting the game.. hold hands up close to the sensors for a specified amount of time to begin
  bool start = false;
  int player1, player2;
  int i = 0;
  int led_c;
  int y_1 = 5;
  int y_2 = 26;
  int dist = 4;
  int count_menu = 0;
  bool indicator = false;

  while(start == false){
    player1 = getTFminiDistance(Serial1)*LIDAR_DIVIDE; //this implementation for getting distance works surprisingly well
    player2 = getTFminiDistance(Serial2)*LIDAR_DIVIDE;

    matrix.clear();
    logo();
    drawVer();
    //draw padels as 1x5 pixels
    for(int x = player1-2; x <= player1+2; x++){
      matrix.drawPixel(x, y_1, matrix.Color(0, 255, 0));
    }
    for(int x = player2-2; x <= player2+2; x++){
      matrix.drawPixel(24-x, y_2, matrix.Color(255, 255, 255));
    }
    if(player1 <= dist && player2 <= dist){
      count_menu = 0;
      indicator = false;
      i++;
      led_c = (int)(i/2.5);
      for(int y = 0; y <= led_c; y++){
        matrix.drawPixel(0, y, matrix.Color(255, 255, 255));
        matrix.drawPixel(23, 32-y, matrix.Color(255, 255, 255));
      }
    }
    else if(player1 > dist || player2 > dist){
      i = 0;
    }
    if(i >= 80){
      start = true;
    }
    count_menu++;
    if(count_menu%50 == 0){
      if(indicator == false){
        indicator = true;
      } else {
        indicator = false;
      }
    }
    if(indicator == true){
      drawStart();
    }
    matrix.show();
  }
  return;
}

void pong(){
  gameOver = false;
  player1_score = 0;
  player2_score = 0;
  while (gameOver == false){
    matrix.clear();
    drawWalls();
    drawScore();
    calcWall();
    enterFrameHandler();
    //First to 7 wins
    if(player1_score == 7 || player2_score == 7){
      gameOver = true;
      if(player1_score == 7){
        winner = 1;
      } else {
        winner = 2;
      }
    }
    //delay(15); //Gamerate
  }
  endScreen();
  return;
}

void endScreen(){
  int end_count = 0;
  int i = 80;
  if (winner == 1){
    while(end_count <= 160){
      matrix.clear();
      //drawWalls();
      if(end_count%2==0) i--;
      int led_c = (int)(i/2.5);
      for(int y = 0; y <= led_c; y++){
        matrix.drawPixel(0, y, matrix.Color(255, 255, 255));
        matrix.drawPixel(23, 32-y, matrix.Color(255, 255, 255));
      }
      test();
      drawPlayer1Win();
      matrix.show();
      end_count++;
      delay(10);
    }
  } else {
    int i = 80;
    while(end_count <= 160){
      matrix.clear();
      //drawWalls();
      if(end_count%2==0) i--;
      int led_c = (int)(i/2.5);
      for(int y = 0; y <= led_c; y++){
        matrix.drawPixel(0, y, matrix.Color(255, 255, 255));
        matrix.drawPixel(23, 32-y, matrix.Color(255, 255, 255));
      }
      test();
      drawPlayer2Win();
      matrix.show();
      end_count++;
      delay(10);
    }
  }
  winner = 0;
}


//model function for LiDAR
void test(){
  //y is fixed for 1 player and the other
  int player1 = getTFminiDistance(Serial1)*LIDAR_DIVIDE;
  int player2 = getTFminiDistance(Serial2)*LIDAR_DIVIDE;
  int y_1 = 5;
  int y_2 = 26;

  //draw padels as 1x5 pixels
  for(int x = player1-2; x <= player1+2; x++){
    matrix.drawPixel(x, y_1, matrix.Color(0, 255, 0));
  }
  for(int x = player2-2; x <= player2+2; x++){
    matrix.drawPixel(24-x, y_2, matrix.Color(255, 255, 255));
  }
}

// DRAW FUNCTIONS \\

void drawWalls(){
  for(int y = 0; y <= 32; y++){
    matrix.drawPixel(0, y, matrix.Color(255, 255, 255));
    matrix.drawPixel(23, y, matrix.Color(255, 255, 255));
  }
}

void drawScore(){
  if(player1_score == 0) p1_0();
  else if(player1_score == 1) p1_1();
  else if(player1_score == 2) p1_2();
  else if(player1_score == 3) p1_3();
  else if(player1_score == 4) p1_4();
  else if(player1_score == 5) p1_5();
  else if(player1_score == 6) p1_6();

  if(player2_score == 0) p2_0();
  else if(player2_score == 1) p2_1();
  else if(player2_score == 2) p2_2();
  else if(player2_score == 3) p2_3();
  else if(player2_score == 4) p2_4();
  else if(player2_score == 5) p2_5();
  else if(player2_score == 6) p2_6();
}

int score_pos_x = 0;
int score_pos_y = 0;
int score_r = 255;
int score_g = 255;
int score_b = 255;

void p1_0(){
  matrix.drawPixel(score_pos_x+4, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  
}

void p1_1(){
  matrix.drawPixel(score_pos_x+3, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
}

void p1_2(){
  matrix.drawPixel(score_pos_x+2, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  
}

void p1_3(){
  matrix.drawPixel(score_pos_x+2, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
}

void p1_4(){
  matrix.drawPixel(score_pos_x+4, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
}

void p1_5(){
  matrix.drawPixel(score_pos_x+4, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
}

void p1_6(){
  matrix.drawPixel(score_pos_x+4, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+0, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+2, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+3, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+4, score_pos_y+3, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score_pos_x+2, score_pos_y+1, matrix.Color(score_r, score_g, score_b));
}

int score2_pos_x = 0;
int score2_pos_y = 0;

void p2_0(){
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
}

void p2_1(){//p2 is high side above logo
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  //Serial.println("p2_1");
}

void p2_2(){
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  //matrix.drawPixel(score2_pos_x+19, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
}

void p2_3(){
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
}

void p2_4(){
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
}

void p2_5(){
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
}

void p2_6(){
  //5
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+28, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+29, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+19, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+20, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+31, matrix.Color(score_r, score_g, score_b));
  //extra for 6
  matrix.drawPixel(score2_pos_x+21, score2_pos_y+30, matrix.Color(score_r, score_g, score_b));
}

int logo_g = 0;
int logo_b = 255;
int logo_r = 196;
int x_pos = 0;
int y_pos = 2;

void drawPlayer1Win(){
  int x_pos_p1w = 0;
  int y_pos_p1w = 0;
  int _r = 0;
  int _g = 255;
  int _b = 0;
  int _rl = 255;
  int _bl = 0;
  int _gl = 0;
  //WIN
  //W
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+3, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+2, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+1, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+7, y_pos_p1w+0, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+8, y_pos_p1w+1, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+0, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+1, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+2, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+3, matrix.Color(_r, _g, _b));
  //I
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+0, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+1, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+2, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+3, matrix.Color(_r, _g, _b));
  //N
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+3, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+2, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+1, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+0, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+3, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+2, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+1, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+0, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+16, y_pos_p1w+1, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+15, y_pos_p1w+2, matrix.Color(_r, _g, _b));
  
  //LOSS
  //L
  matrix.drawPixel(x_pos_p1w+18, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+18, y_pos_p1w+29, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+18, y_pos_p1w+30, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+18, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+16, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  //O
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+29, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+30, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+29, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+30, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+12, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  //S
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+8, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+30, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+8, y_pos_p1w+30, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+8, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+29, matrix.Color(_rl, _gl, _bl));
  //S :: translated by -4
  matrix.drawPixel(x_pos_p1w+10-4, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9-4, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+8-4, y_pos_p1w+28, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9-4, y_pos_p1w+30, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+8-4, y_pos_p1w+30, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+8-4, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9-4, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+10-4, y_pos_p1w+31, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+10-4, y_pos_p1w+29, matrix.Color(_rl, _gl, _bl));
}

void drawPlayer2Win(){
  int x_pos_p1w = 0;
  int y_pos_p1w = 0;
  int _r = 0;
  int _g = 255;
  int _b = 0;
  int _rl = 255;
  int _bl = 0;
  int _gl = 0;
  //WIN
  //W
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+28, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+29, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+17, y_pos_p1w+30, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+16, y_pos_p1w+31, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+15, y_pos_p1w+30, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+15, y_pos_p1w+29, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+31, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+31, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+29, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+28, matrix.Color(_r, _g, _b));
  //I
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+28, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+29, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+30, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+31, matrix.Color(_r, _g, _b));
  //N
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+28, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+29, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+30, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+31, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+28, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+29, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+30, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+31, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+7, y_pos_p1w+30, matrix.Color(_r, _g, _b));
  matrix.drawPixel(x_pos_p1w+8, y_pos_p1w+29, matrix.Color(_r, _g, _b));
  
  //LOSS
  //L
  matrix.drawPixel(x_pos_p1w+5, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+5, y_pos_p1w+1, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+5, y_pos_p1w+2, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+5, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+6, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+7, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  //O
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+1, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+2, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+9, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+10, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+1, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+2, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+11, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  //S
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+15, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+15, y_pos_p1w+1, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+1, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+2, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+13, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+15, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  //S
  matrix.drawPixel(x_pos_p1w+13+4, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14+4, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+15+4, y_pos_p1w+0, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+15+4, y_pos_p1w+1, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14+4, y_pos_p1w+1, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+13+4, y_pos_p1w+2, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+13+4, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+14+4, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
  matrix.drawPixel(x_pos_p1w+15+4, y_pos_p1w+3, matrix.Color(_rl, _gl, _bl));
}

void logo(){ //Printing logo on start up
  // "LiDAR"
  matrix.drawPixel(x_pos+4, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+4, y_pos+19, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+4, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+4, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+5, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+6, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+8, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+8, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+8, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+10, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+11, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+10, y_pos+19, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+12, y_pos+19, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+12, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+10, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+11, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+10, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+14, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+14, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+14, y_pos+19, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+15, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+15, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+16, y_pos+19, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+16, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+16, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+18, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+18, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+18, y_pos+19, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+18, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+19, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+19, y_pos+18, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+20, y_pos+17, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+20, y_pos+19, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+20, y_pos+20, matrix.Color(logo_r, logo_g, logo_b));
  // "PONG"
  matrix.drawPixel(x_pos+5, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+5, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+5, y_pos+14, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+5, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+6, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+6, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+7, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+7, y_pos+14, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+7, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+9, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+9, y_pos+14, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+9, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+9, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+10, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+10, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+11, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+10, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+11, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+11, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+11, y_pos+14, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+11, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  //O end
  //n start
  matrix.drawPixel(x_pos+13, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+13, y_pos+14, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+13, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+13, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+14, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+15, y_pos+14, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+15, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+15, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  //G start
  matrix.drawPixel(x_pos+17, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+17, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+17, y_pos+14, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+18, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+18, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+19, y_pos+12, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+19, y_pos+13, matrix.Color(logo_r, logo_g, logo_b));
  matrix.drawPixel(x_pos+19, y_pos+15, matrix.Color(logo_r, logo_g, logo_b));
}

int start_g = 0;
int start_b = 0;
int start_r = 255;
int x_pos_s = 1; //center V 1.1
int y_pos_s = 0;

void drawStart(){
  //arrows
  matrix.drawPixel(x_pos_s+1, y_pos_s+1, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+2, y_pos_s+0, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+3, y_pos_s+1, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+20, y_pos_s+30, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+21, y_pos_s+31, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+22, y_pos_s+30, matrix.Color(start_r, start_g, start_b));
  //S
  matrix.drawPixel(x_pos_s+3, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+4, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+3, y_pos_s+10, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+4, y_pos_s+9, matrix.Color(start_r, start_g, start_b));
  //T
  matrix.drawPixel(x_pos_s+6, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+7, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+8, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+7, y_pos_s+10, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+7, y_pos_s+9, matrix.Color(start_r, start_g, start_b));
  //A
  matrix.drawPixel(x_pos_s+10, y_pos_s+9, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+10, y_pos_s+10, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+11, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+12, y_pos_s+10, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+12, y_pos_s+9, matrix.Color(start_r, start_g, start_b));
  //R
  matrix.drawPixel(x_pos_s+14, y_pos_s+9, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+14, y_pos_s+10, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+14, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+15, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+15, y_pos_s+10, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+16, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+16, y_pos_s+9, matrix.Color(start_r, start_g, start_b));
  //T
  matrix.drawPixel(x_pos_s+18, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+19, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+20, y_pos_s+11, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+19, y_pos_s+10, matrix.Color(start_r, start_g, start_b));
  matrix.drawPixel(x_pos_s+19, y_pos_s+9, matrix.Color(start_r, start_g, start_b));
}

int ver_r = 255;
int ver_g = 255;
int ver_b = 255;

void drawVer(){
  matrix.drawPixel(x_pos_s+13, y_pos_s+2, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+13, y_pos_s+1, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+14, y_pos_s+0, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+15, y_pos_s+1, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+15, y_pos_s+2, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+17, y_pos_s+2, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+17, y_pos_s+1, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+17, y_pos_s+0, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+19, y_pos_s+0, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+21, y_pos_s+0, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+21, y_pos_s+1, matrix.Color(ver_r, ver_g, ver_b));
  matrix.drawPixel(x_pos_s+21, y_pos_s+2, matrix.Color(ver_r, ver_g, ver_b));
  //matrix.drawPixel(x_pos_s+22, y_pos_s+2, matrix.Color(ver_r, ver_g, ver_b));
  //matrix.drawPixel(x_pos_s+22, y_pos_s+0, matrix.Color(ver_r, ver_g, ver_b));
  //matrix.drawPixel(x_pos_s+23, y_pos_s+0, matrix.Color(ver_r, ver_g, ver_b));
  //matrix.drawPixel(x_pos_s+23, y_pos_s+1, matrix.Color(ver_r, ver_g, ver_b));
  //matrix.drawPixel(x_pos_s+23, y_pos_s+2, matrix.Color(ver_r, ver_g, ver_b));
}
 
void setup(){
  Serial.begin(9600);       //Initialize hardware serial port (serial debug port)
  while (!Serial);          // wait for serial port to connect. Needed for native USB port only
  Serial1.begin(115200);    //Initialize the data rate for LiDAR->Arduino ports
  Serial2.begin(115200);    // "
  matrix.begin();
  LEDbrightness = 10;
  matrix.setBrightness(LEDbrightness);
  easyMode = true;
}
 
void loop(){
  menu();
  pong();
}

//DEMO LUNA WORKING - Put this in loop()
  //
  //   getTFminiData(Serial2, &distance, &strength);
  // while (!distance)
  // {
  //   getTFminiData(Serial2, &distance, &strength);
  //   if (distance)
  //   {
  //     Serial.print(distance);
  //     Serial.print("cm\t");
  //     Serial.print("strength: ");
  //     Serial.println(strength);
  //   }
  // }
  //
