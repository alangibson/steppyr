
def _BV(bit):
  """
  An implementation of the Arduino _BV macro.
  http://www.atmel.com/webdoc/AVRLibcReferenceManual/FAQ_1faq_use_bv.html
  """
  return 1 << bit
