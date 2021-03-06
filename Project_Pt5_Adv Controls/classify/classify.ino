/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                  100
#define PRELENGTH                     20
#define THRESHOLD                     0.4

#define KMEANS_THRESHOLD            0.03
#define LOUDNESS_THRESHOLD          200

/*---------------------------*/
/*      CODE BLOCK PCA2      */
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

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
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

      // Compare 'best_dist' against the 'KMEANS_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'KMEANS_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      if (best_dist < KMEANS_THRESHOLD) {
        Serial.println(best_index);
      } else {
        Serial.println("the recording is noise.");
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }


    delay(2000);
    re_pointer = 0;
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

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
