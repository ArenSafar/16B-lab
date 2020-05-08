/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/


float theta_left = 0.3423;
float theta_right = 0.4389;
float beta_left = -0.2468;
float beta_right = 11.41;
float v_star = 41.3;

// PWM inputs to jolt the car straight
int left_jolt = 250;
int right_jolt= 250;

// Control gains
float k_left = 0.5;
float k_right = 0.5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/
float driveStraight_left(float delta) {
  return (v_star+beta_left)/theta_left - k_left*delta/theta_left;
}

float driveStraight_right(float delta) {
  return (v_star+beta_right)/theta_right + k_right*delta/theta_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 5;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 5000, 2500, 5000};

float delta_reference(int k) {
  // YOUR CODE HERE
  float delta = CAR_WIDTH*v_star*k / (5*TURN_RADIUS);
  if (drive_mode == DRIVE_RIGHT) {
    return -delta;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return delta;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}
/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                  80
#define PRELENGTH                     15
#define THRESHOLD                     0.4

#define KMEANS_THRESHOLD            0.02
#define LOUDNESS_THRESHOLD          200

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/
float pca_vec1[SNIPPET_SIZE] = {-0.04161366419995596, -0.03731327921896732, -0.03968086422852568, -0.04123333329541069, -0.038589514402042085, -0.0400731664364243, -0.036086852292631345, -0.04247136372491747, -0.03747094475399701, -0.03549188788268677, -0.030930987586582057, -0.02316511091475825, -0.016051731776857783, -0.01687516024526242, -0.022016405317544636, -0.05754357530102963, -0.0771799403160633, -0.09950244883407758, -0.11315314304214125, -0.11914628384458087, -0.1383755566685659, -0.13431063235015628, -0.131600787731301, -0.15163548155862866, -0.15015254362102118, -0.1405379185663322, -0.157256941919504, -0.1653923639953437, -0.1507944934730896, -0.14214996449589462, -0.14408342470678956, -0.13432776778428865, -0.11222969762202487, -0.1108357735492631, -0.101555806259732, -0.09027826633869021, -0.06132996423626003, -0.03490208242982447, -0.02589913845104851, -0.017772378452910537, -0.0011070490000163978, 0.01467942247674064, 0.028795509600934757, 0.04755921976724414, 0.05156710535521565, 0.06297307741220241, 0.08033082022591785, 0.10910391035011589, 0.1336935326365124, 0.1551051999801721, 0.17503103683588178, 0.19378999838269773, 0.1903511244042069, 0.19127609125076991, 0.18332992750313049, 0.18770407492695973, 0.1953768043106415, 0.2025641951970631, 0.19984594215662074, 0.18362567117353393, 0.16154576845970142, 0.15532190251421546, 0.15324706444420194, 0.1397781709806165, 0.1412934767114203, 0.11293435988520902, 0.11449925395768253, 0.10311830918078421, 0.052057322557540704, 0.04153688287157927, 0.029379748034017803, 0.011616051126947413, -0.020323139287197617, -0.030467311335292232, -0.0474033111592803, -0.05591928809527999, -0.0722537243441435, -0.10404019198663539, -0.11288765419022756, -0.09761866344727942};
float pca_vec2[SNIPPET_SIZE] = {-0.09171715301915567, -0.09466967027831841, -0.08617980446961315, -0.09019208899227799, -0.07543875680227566, -0.09336453497621847, -0.11322143103288797, -0.08737039506222267, -0.09240920264523085, -0.11587770619791642, -0.1083405494494187, -0.11319759352838936, -0.10267745954683066, -0.08787297680771948, -0.07573770547344723, -0.08788924187701075, -0.13512078522934357, -0.19669613071950837, -0.17148579305872944, -0.1492340914678421, -0.1421655916191803, -0.12136662699327572, -0.08990587890177405, -0.16924036721369365, -0.11518815091245305, -0.07541732848049428, -0.10452357834592975, -0.11899851510683952, -0.00697710970935958, 0.023800906387768803, 0.06816383078414538, 0.19532681776506722, 0.24567816202483855, 0.25406977680618775, 0.2444970678684718, 0.25777757578757154, 0.254895275244066, 0.2313942870546174, 0.18970070752302934, 0.19034316790095954, 0.17289636670233408, 0.1363118769073459, 0.12951011192295422, 0.10625037185429331, 0.09125921117957839, 0.06651022575499048, 0.05289686715378612, 0.01216587512361884, 0.014455687129415914, -0.013510208489177057, -0.033208895412840736, -0.03197297322898732, -0.02315763649142685, -0.02556433979007305, -0.01804883165849191, -0.04080326784471394, -0.035640748870766466, -0.03514217197155697, -0.033160042568239435, -0.01903775380949605, -0.017689527922172963, -0.018213811706856976, -0.0017956965868965353, -0.012828076872152774, -0.02119328656551398, -0.027299795250516082, -0.02626977461864845, 0.002971424544276778, 0.015565869352904096, 0.023658551862895838, 0.03155822955476775, 0.015295214579130354, 0.05963664581365465, 0.023985957450252556, 0.07134698235311972, 0.08922599643826523, 0.049108921511867944, 0.05135718539468796, 0.08090395921839252, 0.09449395062662813};
float pca_vec3[SNIPPET_SIZE] = {-0.00951511590120497, -0.010341396151106505, -0.0042422867044743726, 0.0050293457212697915, 0.004215935661351949, 0.009786542315235902, 0.001469211268956827, 0.011254560779855621, 0.012108876426783863, 0.011034786657979084, 0.00028504101081325387, -0.000263298581230311, 0.012181785850543404, 0.0059539139347261835, 0.016172785096348882, -0.013931227861668744, -0.012842734088498935, -0.021979477859831908, 0.036953842877525504, 0.07614196301179645, 0.05179354082595492, 0.08808014010675237, 0.08357525616386188, 0.03200430041198055, 0.05595307575720654, 0.0808780365937796, 0.09102864159645607, 0.08449084117587008, 0.08480266488794982, 0.11683839744628613, 0.136214536538771, 0.18491451872295417, 0.1776994531405706, 0.17024990698311496, 0.10180239211755467, 0.14106827878797476, 0.11944162717758357, 0.06648113071115805, 0.016283536089361717, 0.005226755351056861, -0.017145799717705107, -0.04191929321009202, -0.025420154406485443, -0.022678568804851057, -0.014115008820635948, 0.012715558722660641, 0.016851780517330914, 0.05911988758590717, 0.06550123811334896, 0.08921171896294262, 0.08935046669382404, 0.12349733994324916, 0.10123762429982462, 0.08621497624060155, 0.08427925390498095, 0.1127415640610481, 0.10869408711298396, 0.09246549512504124, 0.09413428583156873, 0.06721801644647771, 0.046412115833957074, 0.013413401787046258, -0.02833898723674123, -0.035512682179229996, -0.05539881714390499, -0.1200128243278891, -0.12449383414409664, -0.17619675913200136, -0.18569170742534538, -0.23992441317757665, -0.24650989728329434, -0.19848311841638638, -0.24615307630182462, -0.26298375986828665, -0.22705723739832193, -0.21798699836970614, -0.23769463679830552, -0.17869163140652247, -0.18233045760173952, -0.19661923206321988};
float projected_mean_vec[3] = {-0.02768025671907778, 0.0006357634881794727, -0.007785202801245534};
float centroid1[3] = {-0.03855599679705657, -0.030959542898433366, -0.0043249113622259696};
float centroid2[3] = {0.052193753840531466, -0.006661590495182121, 0.013162826425772217};
float centroid3[3] = {0.006547416682492747, 0.011099341964467274, -0.02471312707244916};
float centroid4[3] = {-0.03192331472463397, 0.015559341735308726, 0.012742394093104572};

float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;
      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' on the principal components
      // YOUR CODE HERE
 for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += pca_vec1[i] * result[i] ;
          proj2 += pca_vec2[i] * result[i] ;
          proj3 += pca_vec3[i] * result[i] ;
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];
      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      for (int i=0; i < 4; i++) {
         
           float dist = l2_norm3(proj1, proj2, proj3, centroids[i]);
        Serial.println(dist);
        if (dist < best_dist) {
          best_dist = dist;
          best_index = i;
        }
      }

      // Classification
      // Use the function l2_norm defined above
      // ith centroid: centroids[i]
      // YOUR CODE HERE


      // Check against KMEANS_THRESHOLD and print result over serial
      // YOUR CODE HERE
      if (best_dist < KMEANS_THRESHOLD) {
        drive_mode = best_index ; // from 0-3, inclusive
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TA2CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TA2CCTL0 = CCIE; // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
