using System;
using System.Collections.Generic;
using System.ComponentModel.Composition;
using LabNation.Interfaces;

namespace LabNation.Decoders
{
    [Export(typeof(IDecoder))]
    public class SimpleDecoder : IDecoder
    {
        public DecoderDescription Description => new DecoderDescription()
        {
            Name = "Simple Test Decoder",
            ShortName = "Simple",
            Author = "Debug",
            VersionMajor = 1,
            VersionMinor = 0,
            Description = "A minimal decoder for testing plugin loading.",
            InputWaveformTypes = new Dictionary<string, Type>() { { "Input", typeof(bool) } },
            Parameters = null
        };

        public DecoderOutput[] Process(Dictionary<string, Array> inputWaveforms, Dictionary<string, object> parameters, double samplePeriod)
        {
            // Does nothing, just returns an empty list
            return new List<DecoderOutput>().ToArray();
        }
    }
} 