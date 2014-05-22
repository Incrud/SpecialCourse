/***************************************************/
/*! \class BassGuitar
    \brief STK BassGuitar model class.

    This class implements a BassGuitar model with an arbitrary number of
    strings (specified during instantiation).  Each string is
    represented by an stk::Twang object.  The model supports commuted
    synthesis, as discussed by Smith and Karjalainen.  It also includes
    a basic body coupling model and supports feedback.

    This class does not attempt voice management.  Rather, most
    functions support a parameter to specify a particular string
    number and string (voice) management is assumed to occur
    externally.  Note that this class does not inherit from
    stk::Instrmnt because of API inconsistencies.

    This is a digital waveguide model, making its use possibly subject
    to patents held by Stanford University, Yamaha, and others.

    Control Change Numbers: 
       - Bridge Coupling Gain = 2
       - Pluck Position = 4
       - Loop Gain = 11
       - Coupling Filter Pole = 1
       - Pick Filter Pole = 128

    by Gary P. Scavone, 2012.
*/
/***************************************************/

#include "BassGuitar.h"
#include "FileWvIn.h"
#include "Noise.h"
#include "SKINI.msg"
#include <cmath>

namespace stk {

#define BASE_COUPLING_GAIN 0.01

BassGuitar :: BassGuitar( unsigned int nStrings, std::string bodyfile )
{
  strings_.resize( nStrings );
  stringState_.resize( nStrings, 0 );
  decayCounter_.resize( nStrings, 0 );
  filePointer_.resize( nStrings, 0 );
  pluckGains_.resize( nStrings, 0 );

  setBodyFile( bodyfile );

  couplingGain_ = BASE_COUPLING_GAIN;
  couplingFilter_.setPole( 0.9 );
  pickFilter_.setPole( 0.95 );
  lastFrame_.resize(1, 1, 0.0);
}

void BassGuitar :: clear( void )
{
  for ( unsigned int i=0; i<strings_.size(); i++ ) {
    strings_[i].clear();
    stringState_[i] = 0;
    filePointer_[i] = 0;
  }
}

void BassGuitar :: setBodyFile( std::string bodyfile )
{
  bool fileLoaded = false;
  if ( bodyfile != "" ) {
    try {
      FileWvIn file( bodyfile );
  
      // Fill the StkFrames variable with the (possibly interpolated)
      // file data.
      excitation_.resize( (unsigned long) ( 0.5 + ( file.getSize() * Stk::sampleRate() / file.getFileRate() ) ) );
      file.tick( excitation_ );
      fileLoaded = true;
    }
    catch ( StkError &error ) {
      oStream_ << "BassGuitar::setBodyFile: file error (" << error.getMessage() << ") ... using noise excitation.";
      handleError( StkError::WARNING );
    }
  }

  if ( !fileLoaded ) {
    unsigned int M = 200;  // arbitrary value
    excitation_.resize( M );
    Noise noise;
    noise.tick( excitation_ );
    // Smooth the start and end of the noise.
    unsigned int N = (unsigned int) M * 0.2; // arbitrary value
    for ( unsigned int n=0; n<N; n++ ) {
      StkFloat weight = 0.5 * ( 1.0 - cos( n * PI / (N-1) ) );
      excitation_[n] *= weight;
      excitation_[M-n-1] *= weight;
    }
  }

  // Filter the excitation to simulate pick hardness
  pickFilter_.tick( excitation_ );

  // Compute file mean and remove (to avoid DC bias).
  StkFloat mean = 0.0;
  for ( unsigned int i=0; i<excitation_.frames(); i++ )
    mean += excitation_[i];
  mean /= excitation_.frames();

  for ( unsigned int i=0; i<excitation_.frames(); i++ )
    excitation_[i] -= mean;

  // Reset all the file pointers.
  for ( unsigned int i=0; i<strings_.size(); i++ )
    filePointer_[i] = 0;
}


void BassGuitar :: setPluckPosition( StkFloat position, int string )
{
  if ( position < 0.0 || position > 1.0 ) {
    std::cerr << "BassGuitar::setPluckPosition: position parameter out of range!";
    handleError( StkError::WARNING ); return;
  }

  if ( string >= (int) strings_.size() ) {
    oStream_ << "BassGuitar::setPluckPosition: string parameter is greater than number of strings!";
    handleError( StkError::WARNING ); return;
  }

  if ( string < 0 ) // set all strings
    for ( unsigned int i=0; i<strings_.size(); i++ )
      strings_[i].setPluckPosition( position );
  else
    strings_[string].setPluckPosition( position );
}

void BassGuitar :: setLoopGain( StkFloat gain, int string )
{
  if ( gain < 0.0 || gain > 1.0 ) {
    std::cerr << "BassGuitar::setLoopGain: gain parameter out of range!";
    handleError( StkError::WARNING ); return;
  }

  if ( string >= (int) strings_.size() ) {
    oStream_ << "BassGuitar::setLoopGain: string parameter is greater than number of strings!";
    handleError( StkError::WARNING ); return;
  }

  if ( string < 0 ) // set all strings
    for ( unsigned int i=0; i<strings_.size(); i++ )
      strings_[i].setLoopGain( gain );
  else
    strings_[string].setLoopGain( gain );
}

void BassGuitar :: setFrequency( StkFloat frequency, unsigned int string )
{
#if defined(_STK_DEBUG_)
  if ( frequency <= 0.0 ) {
    oStream_ << "BassGuitar::setFrequency: frequency parameter is less than or equal to zero!";
    handleError( StkError::WARNING ); return;
  }

  if ( string >= strings_.size() ) {
    oStream_ << "BassGuitar::setFrequency: string parameter is greater than number of strings!";
    handleError( StkError::WARNING ); return;
  }
#endif

  strings_[string].setFrequency( frequency );
}

void BassGuitar :: noteOn( StkFloat frequency, StkFloat amplitude, unsigned int string )
{
#if defined(_STK_DEBUG_)
  if ( string >= strings_.size() ) {
    oStream_ << "BassGuitar::noteOn: string parameter is greater than number of strings!";
    handleError( StkError::WARNING ); return;
  }

  if ( Stk::inRange( amplitude, 0.0, 1.0 ) == false ) {
    oStream_ << "BassGuitar::noteOn: amplitude parameter is outside range 0.0 - 1.0!";
    handleError( StkError::WARNING ); return;
  }
#endif

  this->setFrequency( frequency, string );
  stringState_[string] = 2;
  filePointer_[string] = 0;
  strings_[string].setLoopGain( 0.995 );
  pluckGains_[string] = amplitude;
}

void BassGuitar :: noteOff( StkFloat amplitude, unsigned int string )
{
#if defined(_STK_DEBUG_)
  if ( string >= strings_.size() ) {
    oStream_ << "BassGuitar::noteOff: string parameter is greater than number of strings!";
    handleError( StkError::WARNING ); return;
  }

  if ( Stk::inRange( amplitude, 0.0, 1.0 ) == false ) {
    oStream_ << "BassGuitar::noteOff: amplitude parameter is outside range 0.0 - 1.0!";
    handleError( StkError::WARNING ); return;
  }
#endif

  strings_[string].setLoopGain( (1.0 - amplitude) * 0.9 );
  stringState_[string] = 1;
}

void BassGuitar :: controlChange( int number, StkFloat value, int string )
{
#if defined(_STK_DEBUG_)
  if ( Stk::inRange( value, 0.0, 128.0 ) == false ) {
    oStream_ << "BassGuitar::controlChange: value (" << value << ") is out of range!";
    handleError( StkError::WARNING ); return;
  }

  if ( string > 0 && string >= (int) strings_.size() ) {
    oStream_ << "BassGuitar::controlChange: string parameter is greater than number of strings!";
    handleError( StkError::WARNING ); return;
  }
#endif

  StkFloat normalizedValue = value * ONE_OVER_128;
  if ( number == 2 )
    couplingGain_ = 1.5 * BASE_COUPLING_GAIN * normalizedValue;
  else if ( number == __SK_PickPosition_ ) // 4
    this->setPluckPosition( normalizedValue, string );
  else if ( number == __SK_StringDamping_ ) // 11
    this->setLoopGain( 0.97 + (normalizedValue * 0.03), string );
  else if ( number == __SK_ModWheel_ ) // 1
    couplingFilter_.setPole( 0.98 * normalizedValue );
  else if (number == __SK_AfterTouch_Cont_) // 128
    pickFilter_.setPole( 0.95 * normalizedValue );
#if defined(_STK_DEBUG_)
  else {
    oStream_ << "BassGuitar::controlChange: undefined control number (" << number << ")!";
    handleError( StkError::WARNING );
  }
#endif
}

} // stk namespace
