int ir_1 = 2;
int ir_2 = 3;
int ir_3 = 4;
int ir_4 = 5;

int counter1;
int counter2;
float distance;
float cycle;
int oldenc_1 = 0;
int oldenc_2= 0;
int oldenc_3= 0;
int oldenc_4 = 0;

int state1;
int oldstate1;
int state2;
int oldstate2;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(ir_1, INPUT);
pinMode(ir_2, INPUT);
pinMode(ir_3, INPUT);
pinMode(ir_4, INPUT);

int ir1 = digitalRead(ir_1);
  int ir2 = digitalRead(ir_2);
   int ir3 = digitalRead(ir_3);
    int ir4 = digitalRead(ir_4);

// State initializations

if (ir1 == 0){
  if (ir2 == 0){ oldstate1 = 0;}
  else {oldstate1 = 1;}
  
}else {
  if (ir2 == 0){oldstate1 = 2;}
  else {oldstate1 = 3;}
}



if (ir3 == 0){
  if (ir4 == 0){ oldstate2 = 0;}
  else {oldstate2 = 1;}
  
}else {
  if (ir4 == 0){oldstate2 = 2;}
  else {oldstate2 = 3;}
}

}


// Main function

void loop() {
  // put your main code here, to run repeatedly:
  int ir1 = digitalRead(ir_1);
  int ir2 = digitalRead(ir_2);

  int ir3 = digitalRead(ir_3);
  int ir4 = digitalRead(ir_4);

  
  int returnvalue1;
  int returnvalue2;

  
if (ir1 == 0){
  if (ir2 == 0){ state1 = 0;}
  else {state1 = 1;}
  
}else {
  if (ir2 == 0){state1 = 2;}
  else {state1 = 3;}
}



if (ir3 == 0){
  if (ir4 == 0){ state2 = 0;}
  else {state2 = 1;}
  
}else {
  if (ir4 == 0){state2 = 2;}
  else {state2 = 3;}
}



switch (oldstate1){
case 0: 
if (state1 == 1){ returnvalue1 = 1;}
else if (state1 == 2) {returnvalue1 = -1;}
else {returnvalue1 = 0;}
break;
case 1:
if (state1 == 3){ returnvalue1 = 1;}
else if (state1 == 0) {returnvalue1 = -1;}
else {returnvalue1 = 0;}
break;
case 2:
if (state1 == 0){ returnvalue1 = 1;}
else if (state1 == 3) {returnvalue1 = -1;}
else {returnvalue1 = 0;}
break;
case 3:
if (state1 == 2){ returnvalue1 = 1;}
else if (state1 == 1) {returnvalue1 = -1;}
else {returnvalue1 = 0;}
break;
}





switch (oldstate2){
case 0: 
if (state2 == 1){ returnvalue2 = 1;}
else if (state2 == 2) {returnvalue2 = -1;}
else {returnvalue2 = 0;}
break;
case 1:
if (state2 == 3){ returnvalue2 = 1;}
else if (state2 == 0) {returnvalue2 = -1;}
else {returnvalue2 = 0;}
break;
case 2:
if (state2 == 0){ returnvalue2 = 1;}
else if (state2 == 3) {returnvalue2 = -1;}
else {returnvalue2 = 0;}
break;
case 3:
if (state2 == 2){ returnvalue2 = 1;}
else if (state2 == 1) {returnvalue2 = -1;}
else {returnvalue2 = 0;}
break;
}



if (ir1 != oldenc_1)
{
  counter1 = counter1 + returnvalue1;
}

if (ir2 != oldenc_3)
{
  counter2 = counter2 + returnvalue2;
}





    oldstate1 = state1;
    oldstate2 = state2;
    oldenc_1 = ir1;
  oldenc_2 = ir2;
  oldenc_3 = ir3;
  oldenc_4 = ir4;
    
// Serial.print(ir1);
// Serial.println(ir2);
Serial.print(-counter1);
Serial.print("  ");
Serial.println(counter2);

 }
