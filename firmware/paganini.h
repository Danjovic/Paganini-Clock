/***
 *    $$$$$$$\                                         $$\           $$\       
 *    $$  __$$\                                        \__|          \__|      
 *    $$ |  $$ |$$$$$$\   $$$$$$\   $$$$$$\  $$$$$$$\  $$\ $$$$$$$\  $$\       
 *    $$$$$$$  |\____$$\ $$  __$$\  \____$$\ $$  __$$\ $$ |$$  __$$\ $$ |      
 *    $$  ____/ $$$$$$$ |$$ /  $$ | $$$$$$$ |$$ |  $$ |$$ |$$ |  $$ |$$ |      
 *    $$ |     $$  __$$ |$$ |  $$ |$$  __$$ |$$ |  $$ |$$ |$$ |  $$ |$$ |      
 *    $$ |     \$$$$$$$ |\$$$$$$$ |\$$$$$$$ |$$ |  $$ |$$ |$$ |  $$ |$$ |      
 *    \__|      \_______| \____$$ | \_______|\__|  \__|\__|\__|  \__|\__|      
 *                       $$\   $$ |                                            
 *                       \$$$$$$  |                                            
 *                        \______/                                             
 *     $$$$$$\  $$\                     $$\                                    
 *    $$  __$$\ $$ |                    $$ |                                   
 *    $$ /  \__|$$ | $$$$$$\   $$$$$$$\ $$ |  $$\                              
 *    $$ |      $$ |$$  __$$\ $$  _____|$$ | $$  |                             
 *    $$ |      $$ |$$ /  $$ |$$ /      $$$$$$  /                              
 *    $$ |  $$\ $$ |$$ |  $$ |$$ |      $$  _$$<                               
 *    \$$$$$$  |$$ |\$$$$$$  |\$$$$$$$\ $$ | \$$\                              
 *     \______/ \__| \______/  \_______|\__|  \__|                             
 *                                                                             
 *   
 *    Danjovic 2022 - Beltane 2022
 *    Released under GPL 3.0      
 */
 
#ifndef _PAGANINI_H_
#define _PAGANINI_H_

// segment position on the 16 bit shift register

#define _A1   (1<<1 )
#define _A2   (1<<5 )
#define _B    (1<<6 )
#define _C    (1<<11)
#define _D1   (1<<9 )
#define _D2   (1<<13)
#define _E    (1<<14)
#define _F    (1<<3 )
#define _G1   (1<<15)
#define _G2   (1<<7 )
#define _H    (1<<2 )
#define _I    (1<<0 )
#define _J    (1<<4 )
#define _K    (1<<12)
#define _L    (1<<8 )
#define _M    (1<<10)


// 7 segment equivalent digits
#define _0_ (_A1|_A2|_B|_C|_D1|_D2|_E|_F        )
#define _1_ (        _B|_C                      )
#define _2_ (_A1|_A2|_B   |_D1|_D2|_E   |_G1|_G2)
#define _3_ (_A1|_A2|_B|_C|_D1|_D2          |_G2)
#define _4_ (        _B|_C           |_F|_G1|_G2)
#define _5_ (_A1|_A2   |_C|_D1|_D2   |_F|_G1|_G2)
#define _6_ (_A1|_A2   |_C|_D1|_D2|_E|_F|_G1|_G2)
#define _7_ (_A1|_A2|_B|_C                      )
#define _8_ (_A1|_A2|_B|_C|_D1|_D2|_E|_F|_G1|_G2)
#define _9_ (_A1|_A2|_B|_C|_D1|_D2   |_F|_G1|_G2)

// lowercase letters
#define _A_ (              _D1|_D2|_E   |_G1                |_L   )  // a
#define _B_ (                  _D2|_E|_F|_G1                |_L   )  // b
#define _C_ (                  _D2|_E   |_G1                      )  // c
#define _D_ (                  _D2|_E   |_G1       |_I      |_L   )  // d
#define _E_ (                  _D2|_E   |_G1             |_K      )  // e
#define _F_ (    _A2                    |_G1|_G2   |_I      |_L   )  // f
#define _G_ (_A1              |_D2   |_F|_G1       |_I      |_L   )  // g
#define _J_ (                  _D2|_E              |_I      |_L   )  // j
#define _L_ (                  _D2|_E|_F                          )  // l
#define _M_ (           _C        |_E   |_G1|_G2            |_L   )  // m
#define _N_ (                      _E   |_G1                |_L   )  // n
#define _O_ (                  _D2|_E   |_G1                |_L   )  // o
#define _P_ (_A1                  |_E|_F|_G1       |_I            )  // p
#define _R_ (                      _E   |_G1                      )  // r
#define _S_ (_A1              |_D2   |_F|_G1                |_L   )  // s
#define _T_ (                  _D2|_E|_F|_G1                      )  // t
#define _U_ (                  _D2|_E                       |_L   )  // u
#define _V_ (                      _E                    |_K      )  // v
#define _Y_ (                  _D2   |_F|_G1       |_I      |_L   )  // y

// Upcase letters
#define _HOUR_    (        _B|_C        |_E|_F|_G1|_G2                  )  // H
#define _MINUTE_  (        _B|_C        |_E|_F        |_H   |_J         )  // M
#define _YEAR_    (        _B|_C|_D1|_D2   |_F|_G1|_G2                  )  // Y
#define _DAY_     (_A1|_A2|_B|_C|_D1|_D2                 |_I      |_L   )  // D
//#define _MONTH_   (_A1|_A2|_B|_C|_D1|_D2|_E|_F|_G1|_G2|_H|_I|_J|_K|_L|_M)  // m


// other patterns
#define _BLANK_ 0 
#define _FULL_  (_A1|_A2|_B|_C|_D1|_D2|_E|_F|_G1|_G2|_H|_I|_J|_K|_L|_M)
#define _STAR_  (                            _G1|_G2|_H|_I|_J|_K|_L|_M)  


// 7 segment digits on the rightmost part of the 16 segment display 
#define _00_ (    _A2|_B|_C|_D1                     |_I      |_L   )
#define _01_ (        _B|_C                                        )
#define _02_ (    _A2|_B   |_D1              |_G2            |_L   )
#define _03_ (    _A2|_B|_C|_D1              |_G2                  )
#define _04_ (        _B|_C                  |_G2   |_I            )
#define _05_ (    _A2|   _C|_D1              |_G2   |_I            )
#define _06_ (    _A2|   _C|_D1              |_G2   |_I      |_L   )
#define _07_ (    _A2|_B|_C                                        )
#define _08_ (    _A2|_B|_C|_D1              |_G2   |_I      |_L   )
#define _09_ (    _A2|_B|_C|_D1              |_G2   |_I            )

// 7 segment digits on the leftmost part of the 16 segment display 
#define _10_  (                                       _I      |_L   )
#define _20_  (_A1              |_D2|_E   |_G1       |_I            )
#define _30_  (_A1              |_D2      |_G1       |_I      |_L   )
#define _40_  (                         _F|_G1       |_I      |_L   )
#define _50_  (_A1              |_D2   |_F|_G1                |_L   )
#define _60_  (_A1              |_D2|_E|_F|_G1                |_L   )
#define _70_  (_A1                                   |_I      |_L   )
#define _80_  (_A1              |_D2|_E|_F|_G1       |_I      |_L   )
#define _90_  (_A1              |_D2   |_F|_G1       |_I      |_L   )
#define _100_ (_A1              |_D2|_E|_F           |_I      |_L   )

// Season indication on the 16 segment display
#define _Yule___    _I  | _0_
#define _Imbolc_    _J  | _0_ 
#define _Ostara_    _G2 | _0_
#define _Beltane    _M  | _0_
#define _Litha__    _L  | _0_
#define _Lammas_    _K  | _0_
#define _Mabon__    _G1 | _0_
#define _Samhain    _H  | _0_

#define _Yul_s0_    _I 
#define _Yul_s1_    _I | _A2  
#define _Yul_s2_    _I | _A2 | _B  
#define _Yul_s3_    _I | _A2 | _B | _C  
#define _Yul_s4_    _I | _A2 | _B | _C | _D1 
#define _Yul_s5_    _I | _A2 | _B | _C | _D1 | _D2  
#define _Yul_s6_    _I | _A2 | _B | _C | _D1 | _D2 | _E  
#define _Yul_s7_    _I | _A2 | _B | _C | _D1 | _D2 | _E | _F  

#define _Imb_s0_    _J 
#define _Imb_s1_    _J | _B  
#define _Imb_s2_    _J | _B | _C  
#define _Imb_s3_    _J | _B | _C | _D1  
#define _Imb_s4_    _J | _B | _C | _D1 | _D2  
#define _Imb_s5_    _J | _B | _C | _D1 | _D2 | _E  
#define _Imb_s6_    _J | _B | _C | _D1 | _D2 | _E | _F  
#define _Imb_s7_    _J | _B | _C | _D1 | _D2 | _E | _F | _A1 

#define _Ost_s0_    _G2 
#define _Ost_s1_    _G2 | _C  
#define _Ost_s2_    _G2 | _C | _D1  
#define _Ost_s3_    _G2 | _C | _D1 | _D2  
#define _Ost_s4_    _G2 | _C | _D1 | _D2 | _E  
#define _Ost_s5_    _G2 | _C | _D1 | _D2 | _E | _F  
#define _Ost_s6_    _G2 | _C | _D1 | _D2 | _E | _F | _A1  
#define _Ost_s7_    _G2 | _C | _D1 | _D2 | _E | _F | _A1 | _A2 

#define _Bel_s0_    _M 
#define _Bel_s1_    _M | _D1  
#define _Bel_s2_    _M | _D1 | _D2  
#define _Bel_s3_    _M | _D1 | _D2 | _E 
#define _Bel_s4_    _M | _D1 | _D2 | _E | _F  
#define _Bel_s5_    _M | _D1 | _D2 | _E | _F | _A1  
#define _Bel_s6_    _M | _D1 | _D2 | _E | _F | _A1 | _A2  
#define _Bel_s7_    _M | _D1 | _D2 | _E | _F | _A1 | _A2 | _B

#define _Lit_s0_    _L 
#define _Lit_s1_    _L | _D2  
#define _Lit_s2_    _L | _D2 | _E  
#define _Lit_s3_    _L | _D2 | _E | _F  
#define _Lit_s4_    _L | _D2 | _E | _F | _A1  
#define _Lit_s5_    _L | _D2 | _E | _F | _A1 | _A2  
#define _Lit_s6_    _L | _D2 | _E | _F | _A1 | _A2 | _B  
#define _Lit_s7_    _L | _D2 | _E | _F | _A1 | _A2 | _B | _C  
 
#define _Lam_s0_    _K 
#define _Lam_s1_    _K | _E  
#define _Lam_s2_    _K | _E | _F  
#define _Lam_s3_    _K | _E | _F | _A1  
#define _Lam_s4_    _K | _E | _F | _A1 | _A2  
#define _Lam_s5_    _K | _E | _F | _A1 | _A2 | _B  
#define _Lam_s6_    _K | _E | _F | _A1 | _A2 | _B | _C  
#define _Lam_s7_    _K | _E | _F | _A1 | _A2 | _B | _C | _D1  
 
#define _Mab_s0_    _G1 
#define _Mab_s1_    _G1 | _F  
#define _Mab_s2_    _G1 | _F | _A1  
#define _Mab_s3_    _G1 | _F | _A1 | _A2  
#define _Mab_s4_    _G1 | _F | _A1 | _A2 | _B  
#define _Mab_s5_    _G1 | _F | _A1 | _A2 | _B | _C  
#define _Mab_s6_    _G1 | _F | _A1 | _A2 | _B | _C | _D1  
#define _Mab_s7_    _G1 | _F | _A1 | _A2 | _B | _C | _D1 | _D2 
 
#define _Sam_s0_    _H  
#define _Sam_s1_    _H | _A1  
#define _Sam_s2_    _H | _A1 | _A2  
#define _Sam_s3_    _H | _A1 | _A2 | _B  
#define _Sam_s4_    _H | _A1 | _A2 | _B | _C  
#define _Sam_s5_    _H | _A1 | _A2 | _B | _C | _D1  
#define _Sam_s6_    _H | _A1 | _A2 | _B | _C | _D1 | _D2  
#define _Sam_s7_    _H | _A1 | _A2 | _B | _C | _D1 | _D2 | _E   

#endif
