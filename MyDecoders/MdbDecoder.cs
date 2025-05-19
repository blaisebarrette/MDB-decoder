///////////////////
//
//  UART/RS232 decoder by Robert44
//  https://github.com/robert44/decoders
//

namespace LabNation.Decoders
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel.Composition;
    using System.Diagnostics;
    using System.Linq;
    using System.Text;

    using LabNation.Interfaces;

    /// <summary>
    /// The MDB decoder.
    /// </summary>
    [Export(typeof(IProcessor))]
    public class DecoderMDB : IDecoder
    {
        /// <summary>
        /// Gets the description.
        /// </summary>
        public DecoderDescription Description
        {
            get
            {
                return new DecoderDescription
                {
                    Name = "MDB decoder",
                    ShortName = "MDB",
                    Author = "Blaise",
                    VersionMajor = 0,
                    VersionMinor = 1,
                    Description = "Multi-Drop Bus (MDB) protocol decoder for vending machine peripherals.",
                    InputWaveformTypes = new Dictionary<string, Type> { { "Input", typeof(bool) } },
                    // MDB has fixed settings (9600 baud, 8 data bits, 1 mode bit, 1 stop bit, no parity)
                    // Parameters might be added later for filtering specific device addresses, etc.
                    Parameters = new DecoderParameter[]
                    {
                       // Removed UART specific parameters like Baud, Bits, Parity, Stopbits, Mode
                    }
                };
            }
        }

        /// <summary>
        /// The decoding method.
        /// </summary>
        /// <param name="inputWaveforms"> The input waveforms. </param>
        /// <param name="parameters"> The parameters. </param>
        /// <param name="samplePeriod"> The sample period. </param>
        /// <returns> The output returned to the scope. </returns>
        public DecoderOutput[] Process(Dictionary<string, Array> inputWaveforms, Dictionary<string, object> parameters, double samplePeriod)
        {
            var decoderOutputList = new List<DecoderOutput>();
            const int MDB_BAUDRATE = 9600;
            // MDB timings (from Référence MDB.md)
            const double T_INTER_BYTE_MAX_MS = 2.0; // Max inter-byte time during send - Increased threshold to 2.0ms
            const double T_RESPONSE_MAX_MS = 5.0;   // Max response time for peripheral

            try
            {
                // Back to original logic expecting bool[] directly
                var serialData = (bool[])inputWaveforms["Input"]; // ORIGINAL
                if (serialData == null || serialData.Length == 0) // Check the input array
                    return decoderOutputList.ToArray();

                // MDB uses Idle High, Active Low (implied from Break condition description)
                bool idleState = true; // Assume Idle High
                bool startBitValue = !idleState; // Start bit is low

                // Calculate samples per bit based on fixed MDB baud rate
                double samplesPerBitExact = 1.0 / (samplePeriod * MDB_BAUDRATE);
                if (samplesPerBitExact < 2) // Need at least 2 samples to reliably detect bit center
                {
                    // Handle error - Sample rate too low for MDB
                    // Maybe add a DecoderOutputError here?
                    return decoderOutputList.ToArray();
                }
                int samplesPerBit = (int)Math.Round(samplesPerBitExact);
                int samplePointOffset = samplesPerBit / 2; // Sample roughly in the middle of the bit

                // MDB Frame: 1 Start, 8 Data, 1 Mode, 1 Stop = 11 bits total
                int frameBitCount = 11;
                int frameSampleCount = frameBitCount * samplesPerBit;

                List<MdbByte> decodedBytes = new List<MdbByte>();
                int currentIndex = 0;

                // ---- State Machine for Byte Detection ----
                // State 0: Idle (Waiting for Start Bit - falling edge)
                // State 1: In Frame (Sampling bits)
                int state = 0;
                int frameStartIndex = 0;

                while (currentIndex < serialData.Length - 1)
                {
                    if (state == 0) // Waiting for Start Bit
                    {
                        // Look for falling edge (Idle High -> Start Low)
                        if (serialData[currentIndex] == idleState && serialData[currentIndex + 1] == startBitValue)
                        {
                            frameStartIndex = currentIndex + 1; // Start bit begins here
                            // Check if there's enough data left for a full frame
                            if (frameStartIndex + frameSampleCount <= serialData.Length)
                            {
                                state = 1; // Move to In Frame state
                                currentIndex = frameStartIndex + samplePointOffset; // Move to first data bit sample point
                            }
                            else
                            {
                                // Not enough data for a full frame, stop searching
                                break;
                            }
                        }
                        else
                        {
                            currentIndex++; // Keep searching for start bit
                        }
                    }
                    else if (state == 1) // In Frame
                    {
                        byte currentByteValue = 0;
                        bool modeBit = false;
                        bool stopBitOk = false;
                        int bitErrors = 0;

                        // --- Corrected Sample Point Calculation ---
                        // Calculate the index where the first data bit (Bit 0) *starts*
                        int startOfBit0Index = frameStartIndex + samplesPerBit;

                        // Sample 8 Data Bits (LSB first)
                        for (int bit = 0; bit < 8; bit++)
                        {
                            // Calculate the center sample point for the current data bit
                            int sampleIndex = startOfBit0Index + (bit * samplesPerBit) + samplePointOffset;
                            if (sampleIndex >= serialData.Length) goto EndProcessing; // Check bounds

                            if (serialData[sampleIndex] == idleState) // Bit is high (1)
                                currentByteValue |= (byte)(1 << bit);
                            // else bit is low (0) - already initialized
                        }

                        // Sample Mode Bit (9th bit after start bit)
                        // Center is 8.5 bit times after start of Bit 0
                        int modeBitSampleIndex = startOfBit0Index + (8 * samplesPerBit) + samplePointOffset;
                        if (modeBitSampleIndex >= serialData.Length) goto EndProcessing; // Check bounds
                        modeBit = (serialData[modeBitSampleIndex] == idleState); // High level == 1

                        // Check Stop Bit (10th bit after start bit)
                        // Center is 9.5 bit times after start of Bit 0
                        int stopBitSampleIndex = startOfBit0Index + (9 * samplesPerBit) + samplePointOffset;
                        stopBitOk = false; // Assume error until proven otherwise
                        if (stopBitSampleIndex >= 0 && stopBitSampleIndex < serialData.Length)
                        {
                             stopBitOk = (serialData[stopBitSampleIndex] == idleState); // Stop bit should be high (idle)
                        }
                        // --- End Corrected Sample Point Calculation ---

                        /* --- Original Sampling Logic (Commented Out) ---
                        for (int bit = 0; bit < 8; bit++)
                        {
                            int sampleIndex = frameStartIndex + samplePointOffset + (bit * samplesPerBit);
                            if (sampleIndex >= serialData.Length) goto EndProcessing; // Check bounds

                            if (serialData[sampleIndex] != startBitValue) // Bit is high (1)
                                currentByteValue |= (byte)(1 << bit);
                        }
                        int modeBitSampleIndex = frameStartIndex + samplePointOffset + (8 * samplesPerBit);
                        if (modeBitSampleIndex >= serialData.Length) goto EndProcessing; // Check bounds
                        modeBit = (serialData[modeBitSampleIndex] != startBitValue);
                        int stopBitSampleIndex = frameStartIndex + samplePointOffset + (10 * samplesPerBit);
                        stopBitOk = false; // Assume error until proven otherwise
                        if (stopBitSampleIndex >= 0 && stopBitSampleIndex < serialData.Length)
                        {
                             stopBitOk = (serialData[stopBitSampleIndex] == idleState);
                        }
                        */

                        if (stopBitOk)
                        {
                            decodedBytes.Add(new MdbByte(frameStartIndex, frameSampleCount, currentByteValue, modeBit));
                        }
                        else
                        {
                            // Framing error - could log this or add specific error output
                             decoderOutputList.Add(new DecoderOutputEvent(
                                frameStartIndex,
                                frameStartIndex + frameSampleCount,
                                DecoderOutputColor.Red,
                                "Framing Error"));
                        }

                        // Move currentIndex to the end of the stop bit, ready for next idle/start bit search
                        currentIndex = frameStartIndex + frameSampleCount;
                        state = 0; // Go back to Idle state
                    }
                }

                EndProcessing:;

                // ---- Block Assembly and Interpretation ----
                if (decodedBytes.Count > 0)
                {
                    List<MdbBlock> blocks = AssembleBlocks(decodedBytes, samplePeriod, T_INTER_BYTE_MAX_MS);
                    ProcessBlocks(blocks, decoderOutputList);
                }
            }
            catch (Exception e)
            {
                // Log the exception or add a DecoderOutputError
                 decoderOutputList.Add(new DecoderOutputEvent(0, 0, DecoderOutputColor.Red, "Decoder Error: " + e.Message));
                Debug.WriteLine("MDB Decoder Exception: " + e);
            }

            return decoderOutputList.ToArray();
        }

        // ---- Helper Methods for MDB ----

        private static readonly Dictionary<byte, string> GeneralPeripheralResponses = new Dictionary<byte, string>
        {
            { 0x00, "JUST RESET" },
            // ACK and NAK are handled separately due to their commonality and single-byte nature
            // { 0x01, "ACK" }, // Typically handled as ACK
            // { 0xFF, "NAK" }, // Typically handled as NAK
            { 0x0B, "COMMAND OUT OF SEQUENCE (Cashless)" }, // Specific to Cashless, but good to have
            { 0x0A, "MALFUNCTION / ERROR" }
            // Other specific responses would be too numerous to list here generically
        };

        private static readonly Dictionary<string, Dictionary<byte, string>> MainCommands = new Dictionary<string, Dictionary<byte, string>>
        {
            {
                "Changer", new Dictionary<byte, string>
                {
                    { 0x08, "RESET" }, { 0x09, "SETUP" }, { 0x0A, "TUBE STATUS" }, { 0x0B, "POLL" },
                    { 0x0C, "COIN TYPE" }, { 0x0D, "DISPENSE" }, { 0x0E, "EXTENSION" }, { 0x0F, "EXPANSION" }
                }
            },
            {
                "Bill Validator", new Dictionary<byte, string>
                {
                    { 0x30, "RESET" }, { 0x31, "SETUP" }, { 0x32, "SECURITY" }, { 0x33, "POLL" },
                    { 0x34, "BILL TYPE" }, { 0x35, "ESCROW" }, { 0x36, "STACKER" }, { 0x37, "EXPANSION" }
                }
            },
            {
                "Cashless #1", new Dictionary<byte, string>
                {
                    { 0x10, "RESET" }, { 0x11, "SETUP" }, { 0x12, "POLL" }, { 0x13, "VEND" },
                    { 0x14, "READER" }, { 0x15, "REVALUE" }, { 0x16, "READER ENABLE" }, { 0x17, "EXPANSION" }
                }
            },
            {
                "Cashless #2", new Dictionary<byte, string>
                {
                    { 0x60, "RESET" }, { 0x61, "SETUP" }, { 0x62, "POLL" }, { 0x63, "VEND" },
                    { 0x64, "READER" }, { 0x65, "REVALUE" }, { 0x66, "READER ENABLE" }, { 0x67, "EXPANSION" }
                }
            },
            {
                "Dispenser #1", new Dictionary<byte, string> // Coin Hopper/Tube
                {
                    { 0x58, "RESET" }, { 0x59, "SETUP" }, { 0x5A, "DISPENSER STATUS" }, { 0x5B, "POLL" },
                    { 0x5C, "MANUAL DISPENSE ENABLE" }, { 0x5D, "DISPENSE" }, { 0x5E, "PAYOUT" }, { 0x5F, "EXPANSION" }
                }
            },
            {
                "Dispenser #2", new Dictionary<byte, string> // Coin Hopper/Tube
                {
                    { 0x70, "RESET" }, { 0x71, "SETUP" }, { 0x72, "DISPENSER STATUS" }, { 0x73, "POLL" },
                    { 0x74, "MANUAL DISPENSE ENABLE" }, { 0x75, "DISPENSE" }, { 0x76, "PAYOUT" }, { 0x77, "EXPANSION" }
                }
            }
            // Other peripherals can be added here
        };

        private static readonly Dictionary<string, Dictionary<byte, Dictionary<byte, string>>> SubCommands = new Dictionary<string, Dictionary<byte, Dictionary<byte, string>>>
        {
            {
                "Changer", new Dictionary<byte, Dictionary<byte, string>>
                {
                    { 0x0F, new Dictionary<byte, string> { { 0x00, "IDENTIFICATION" }, { 0x07, "SEND CTRL MANUAL PAYOUT REPORT" } } } // EXPANSION
                    // DISPENSE (0x0D) might have subcommands (e.g. value) but it's more like data
                }
            },
            {
                "Bill Validator", new Dictionary<byte, Dictionary<byte, string>>
                {
                    {
                        0x37, new Dictionary<byte, string> // EXPANSION
                        {
                            { 0x00, "LVL1 ID NO OPT" }, { 0x01, "FEATURE ENABLE" }, { 0x02, "LVL2+ ID W/ OPT" },
                            { 0x03, "RECYCLER SETUP" }, { 0x05, "BILL DISPENSE STATUS" }, { 0x06, "DISPENSE BILL" },
                            { 0x07, "DISPENSE VALUE" }, { 0x08, "PAYOUT STATUS" }, { 0x09, "PAYOUT VALUE POLL" },
                            { 0x0A, "PAYOUT CANCEL" }, { 0xFA, "FTL REQ TO RCV" }, { 0xFE, "FTL REQ TO SEND" },
                            { 0xFF, "DIAGNOSTICS" }
                        }
                    }
                }
            },
            {
                "Cashless #1", new Dictionary<byte, Dictionary<byte, string>>
                {
                    {
                        0x11, new Dictionary<byte, string> // SETUP
                        { { 0x00, "Config Data" }, { 0x01, "Max/Min Prices" } }
                    },
                    {
                        0x13, new Dictionary<byte, string> // VEND
                        {
                            { 0x00, "Vend Request" }, { 0x01, "Vend Cancel" }, { 0x04, "Session Complete" },
                            { 0x05, "Cash Sale" }, { 0x06, "Negative Vend Request" }, { 0x08, "Coupon Reply" }
                        }
                    },
                    {
                        0x15, new Dictionary<byte, string> // REVALUE
                        { { 0x00, "Revalue Request" }, { 0x01, "Revalue Limit Request" } }
                    },
                    {
                        0x17, new Dictionary<byte, string> // EXPANSION
                        {
                            { 0x00, "Expansion Req ID" }, { 0x04, "Opt Feature Bit Enable" }, { 0xFA, "FTL REQ TO RCV" },
                            { 0xFE, "FTL REQ TO SEND" }, { 0xFF, "DIAGNOSTICS" }
                        }
                    }
                }
            },
            {
                "Cashless #2", new Dictionary<byte, Dictionary<byte, string>> // Similar to Cashless #1 but with 0x60 base
                {
                    {
                        0x61, new Dictionary<byte, string> // SETUP
                        { { 0x00, "Config Data" }, { 0x01, "Max/Min Prices" } }
                    },
                    {
                        0x63, new Dictionary<byte, string> // VEND
                        {
                            { 0x00, "Vend Request" }, { 0x01, "Vend Cancel" }, { 0x04, "Session Complete" },
                            { 0x05, "Cash Sale" }, { 0x06, "Negative Vend Request" }, { 0x08, "Coupon Reply" }
                        }
                    },
                    {
                        0x65, new Dictionary<byte, string> // REVALUE
                        { { 0x00, "Revalue Request" }, { 0x01, "Revalue Limit Request" } }
                    },
                    {
                        0x67, new Dictionary<byte, string> // EXPANSION
                        {
                            { 0x00, "Expansion Req ID" }, { 0x04, "Opt Feature Bit Enable" }, { 0xFA, "FTL REQ TO RCV" },
                            { 0xFE, "FTL REQ TO SEND" }, { 0xFF, "DIAGNOSTICS" }
                        }
                    }
                }
            },
            {
                "Dispenser #1", new Dictionary<byte, Dictionary<byte, string>>
                {
                    // DISPENSE (0x5D) uses data byte for value, not typical sub-command
                    // PAYOUT (0x5E)
                     { 0x5E, new Dictionary<byte, string> { { 0x00, "STATUS" } } }, // PAYOUT
                    {
                        0x5F, new Dictionary<byte, string> // EXPANSION
                        {
                            { 0x00, "IDENTIFICATION" }, { 0x01, "FEATURE ENABLE" }, { 0xFA, "FTL REQ TO RCV" },
                            { 0xFE, "FTL REQ TO SEND" }, { 0xFF, "DIAGNOSTICS" }
                        }
                    }
                }
            },
            {
                "Dispenser #2", new Dictionary<byte, Dictionary<byte, string>>
                {
                    // DISPENSE (0x75) uses data byte for value
                    // PAYOUT (0x76)
                    { 0x76, new Dictionary<byte, string> { { 0x00, "STATUS" } } }, // PAYOUT
                    {
                        0x77, new Dictionary<byte, string> // EXPANSION
                        {
                            { 0x00, "IDENTIFICATION" }, { 0x01, "FEATURE ENABLE" }, { 0xFA, "FTL REQ TO RCV" },
                            { 0xFE, "FTL REQ TO SEND" }, { 0xFF, "DIAGNOSTICS" }
                        }
                    }
                }
            }
        };

        private static string GetMdbCommandDetails(MdbByte addressByte, MdbByte? subCommandByte, string peripheralType)
        {
            StringBuilder commandDetails = new StringBuilder();
            byte peripheralAddrBase = (byte)(addressByte.Value & 0xF8);
            byte cmdCodeOnly = (byte)(addressByte.Value); // Full byte for command lookup

            if (MainCommands.TryGetValue(peripheralType, out var commands) && commands.TryGetValue(cmdCodeOnly, out var commandName))
            {
                commandDetails.Append($"{commandName}");

                // Check for sub-commands
                if (subCommandByte != null && !subCommandByte.ModeBit) // SubCommand byte must have ModeBit = 0
                {
                    if (SubCommands.TryGetValue(peripheralType, out var peripheralSubCommands) &&
                        peripheralSubCommands.TryGetValue(cmdCodeOnly, out var specificSubCommands) &&
                        specificSubCommands.TryGetValue(subCommandByte.Value, out var subCommandName))
                    {
                        commandDetails.Append($": {subCommandName}");
                    }
                    else if (commandName == "DISPENSE" || commandName == "PAYOUT") // Common commands with data bytes not strictly "subcommands"
                    {
                        // For commands like DISPENSE, the subcommand byte is actually data (e.g., amount)
                        // We can just show its value or handle it as "Data" if no specific name.
                        // commandDetails.Append($" (Data: 0x{subCommandByte.Value.Value:X2})");
                        // Or, we might not append anything here if the next byte's detail will show "Data"
                    }
                }
            }
            else
            {
                commandDetails.Append($"Cmd 0x{cmdCodeOnly:X2}"); // Fallback
            }
            return commandDetails.ToString();
        }

        private static string GetPeripheralResponseName(byte responseByteValue, string peripheralType)
        {
            // Specific peripheral responses could be added here if needed, similar to MainCommands
            // For now, using a general list
            if (GeneralPeripheralResponses.TryGetValue(responseByteValue, out var responseName))
            {
                return responseName;
            }
            return $"Resp 0x{responseByteValue:X2}"; // Fallback
        }

        /// <summary>
        /// Represents a single decoded MDB byte with its mode bit.
        /// </summary>
        private class MdbByte
        {
            public int StartIndex { get; }
            public int SampleCount { get; }
            public byte Value { get; }
            public bool ModeBit { get; }

            public int EndIndex => StartIndex + SampleCount;

            public MdbByte(int startIndex, int sampleCount, byte value, bool modeBit)
            {
                StartIndex = startIndex;
                SampleCount = sampleCount;
                Value = value;
                ModeBit = modeBit;
            }

             public override string ToString() => $"0x{Value:X2} (Mode={(ModeBit ? 1:0)}) @{StartIndex}";
        }

        /// <summary>
        /// Represents a block of MDB communication (Master->Peripheral or Peripheral->Master).
        /// </summary>
        private class MdbBlock
        {
            public List<MdbByte> Bytes { get; } = new List<MdbByte>();
            public bool IsMasterToPeripheral { get; private set; } = false;
            public int StartIndex => Bytes.FirstOrDefault()?.StartIndex ?? 0;
            public int EndIndex => Bytes.LastOrDefault()?.EndIndex ?? 0;

            public MdbBlock(MdbByte firstByte)
            {
                Bytes.Add(firstByte);
                // Tentatively determine direction based on the first byte's mode bit
                // Mode=1 for the first byte usually means Master->Peripheral (Address byte)
                // But ACK/NAK from Peripheral also have Mode=1. Refine later if needed.
                IsMasterToPeripheral = firstByte.ModeBit && firstByte.Value != 0x00 && firstByte.Value != 0xFF;
            }

            public void AddByte(MdbByte mdbByte)
            {
                Bytes.Add(mdbByte);
            }

            public override string ToString() => $"{(IsMasterToPeripheral ? "M->P":"P->M")} Block ({Bytes.Count} bytes) @{StartIndex}";

        }


        /// <summary>
        /// Assembles individual MDB bytes into communication blocks based on timing.
        /// </summary>
        private List<MdbBlock> AssembleBlocks(List<MdbByte> bytes, double samplePeriod, double maxInterByteTimeMs)
        {
            List<MdbBlock> blocks = new List<MdbBlock>();
            if (bytes.Count == 0) return blocks;

            double maxInterByteSamples = (maxInterByteTimeMs / 1000.0) / samplePeriod;

            MdbBlock currentBlock = new MdbBlock(bytes[0]);
            blocks.Add(currentBlock);

            for (int i = 1; i < bytes.Count; i++)
            {
                double samplesBetween = bytes[i].StartIndex - bytes[i-1].EndIndex;
                if (samplesBetween <= maxInterByteSamples)
                {
                    currentBlock.AddByte(bytes[i]);
                }
                else
                {
                    // Start a new block
                    currentBlock = new MdbBlock(bytes[i]);
                    blocks.Add(currentBlock);
                }
            }
            return blocks;
        }

         /// <summary>
        /// Processes assembled MDB blocks, calculates checksums, and generates decoder output.
        /// </summary>
        private void ProcessBlocks(List<MdbBlock> blocks, List<DecoderOutput> outputList)
        {
            foreach (var block in blocks)
            {
                if (block.Bytes.Count == 0) continue;

                MdbByte firstByte = block.Bytes[0];
                int blockStartIndex = block.StartIndex;
                int blockEndIndex = block.EndIndex;

                // --- Handle Single-Byte Peripheral Responses (ACK/NAK) ---
                if (block.Bytes.Count == 1 && firstByte.ModeBit)
                {
                    if (firstByte.Value == 0x00) // ACK
                    {
                        outputList.Add(new DecoderOutputEvent(blockStartIndex, blockEndIndex, DecoderOutputColor.Green, "ACK"));
                        continue; // Processed this block
                    }
                    else if (firstByte.Value == 0xFF) // NAK
                    {
                        outputList.Add(new DecoderOutputEvent(blockStartIndex, blockEndIndex, DecoderOutputColor.Red, "NAK"));
                        continue; // Processed this block
                    }
                    // If single byte, Mode=1, but not ACK/NAK, it used to add a "???" event here.
                    // That event is redundant because these responses will be handled by the main processing loop below,
                    // which provides a more detailed DecoderOutputValueNumeric using GetPeripheralResponseName.
                    // Thus, the line causing "???" for these cases is removed.
                }

                // --- Handle Multi-Byte Blocks (Master->Peripheral or Peripheral->Master Data) ---
                if (block.Bytes.Count > 1)
                {
                    byte calculatedChk = 0;
                    MdbByte lastByte = block.Bytes.Last();
                    MdbByte potentialChkByte = lastByte; // In MDB, CHK is the last byte visually

                    // Determine actual data bytes and CHK byte
                    List<MdbByte> dataBytesForChk = new List<MdbByte>();
                    bool checksumExpected = true;

                    if (block.IsMasterToPeripheral)
                    {
                         // M->P: CHK = SUM(Address + Data Bytes)
                         // Address byte is firstByte, potentialChkByte is lastByte
                         if (firstByte.ModeBit && !potentialChkByte.ModeBit) // Valid M->P structure
                         {
                             dataBytesForChk.Add(firstByte); // Include Address byte
                             for(int i=1; i < block.Bytes.Count -1; i++) // Add data bytes
                             {
                                 dataBytesForChk.Add(block.Bytes[i]);
                             }
                         } else {
                             checksumExpected = false; // Invalid structure for checksum
                         }
                    }
                    else // Peripheral->Master
                    {
                        // P->M: CHK = SUM(Data Bytes)
                        // Mode=1 on last *data* byte, CHK follows (Mode=0)
                        checksumExpected = false; // Assume no checksum until proven otherwise
                        int lastDataByteIndex = -1;
                        for(int i=0; i < block.Bytes.Count; i++)
                        {
                            if (block.Bytes[i].ModeBit) {
                                lastDataByteIndex = i;
                                        break;
                                }
                            }

                        // Check if CHK byte exists after the last data byte
                        if (lastDataByteIndex != -1 && lastDataByteIndex < block.Bytes.Count - 1)
                        {
                            potentialChkByte = block.Bytes[lastDataByteIndex + 1];
                            if (!potentialChkByte.ModeBit) // CHK byte should have Mode=0
                            {
                                checksumExpected = true;
                                for(int i=0; i <= lastDataByteIndex; i++) // Sum data bytes up to the one with Mode=1
                                {
                                    dataBytesForChk.Add(block.Bytes[i]);
                                }
                            }
                        }

                    }

                    // Calculate Checksum if expected
                    if (checksumExpected)
                    {
                         foreach (var b in dataBytesForChk)
                         {
                             calculatedChk = (byte)(calculatedChk + b.Value); // Modulo 256 sum
                         }
                    }

                    // --- Generate Output Events for the Block ---
                    DecoderOutputColor blockColor = block.IsMasterToPeripheral ? DecoderOutputColor.Blue : DecoderOutputColor.Green;
                    // We will now output each byte individually
                    // Remove the old block-level label/tooltip builders
                    // StringBuilder blockLabel = new StringBuilder(directionPrefix);
                    // StringBuilder blockTooltip = new StringBuilder();

                    // Keep track of checksum validity
                    bool chkOk = false;
                    if (checksumExpected)
                    {
                        chkOk = (calculatedChk == potentialChkByte.Value);
                    }

                    string peripheralTypeForBlock = "Unknown";
                    if (block.IsMasterToPeripheral && block.Bytes.Count > 0)
                    {
                        peripheralTypeForBlock = GetPeripheralType(block.Bytes[0].Value);
                    }

                    for(int i=0; i < block.Bytes.Count; i++)
                    {
                        MdbByte currentByte = block.Bytes[i];
                        string detail = string.Empty;

                        bool isAddrByte = i == 0 && block.IsMasterToPeripheral && currentByte.ModeBit;
                        MdbByte? subCmdByteCandidate = null;
                        if (isAddrByte && i + 1 < block.Bytes.Count && !block.Bytes[i+1].ModeBit)
                        {
                            subCmdByteCandidate = block.Bytes[i+1];
                        }
                        
                        bool isSubCmdDataByte = i == 1 && block.IsMasterToPeripheral && !currentByte.ModeBit && 
                                              (MainCommands.ContainsKey(peripheralTypeForBlock) && 
                                               SubCommands.ContainsKey(peripheralTypeForBlock) && 
                                               SubCommands[peripheralTypeForBlock].ContainsKey(block.Bytes[0].Value));

                        bool isChk = i == block.Bytes.Count - 1 && checksumExpected && !currentByte.ModeBit;
                        // Determine if this is the last data byte in a P->M message before CHK
                        bool isLastDataInPM = false;
                        if (!block.IsMasterToPeripheral && checksumExpected && dataBytesForChk.Count > 0)
                        {
                            if (dataBytesForChk.Last() == currentByte && currentByte.ModeBit)
                                isLastDataInPM = true;
                        }
                        
                        bool isGenericData = !(isAddrByte || isSubCmdDataByte || isChk || isLastDataInPM);

                        DecoderOutputColor byteColor = blockColor; // Default to block color

                        if (isAddrByte)
                        {
                            string commandStr = GetMdbCommandDetails(currentByte, subCmdByteCandidate, peripheralTypeForBlock);
                            detail = $"Addr ({peripheralTypeForBlock}) - {commandStr}";
                            byteColor = DecoderOutputColor.DarkBlue;
                        }
                        else if (isSubCmdDataByte)
                        {
                            // The command detail is on the AddrByte, this byte is the SubCmd data itself
                            // We can retrieve the sub-command name again if needed for this specific byte, or just label it generically
                            string subCmdName = "SubCmd Data"; // Default
                            if (SubCommands.TryGetValue(peripheralTypeForBlock, out var peripheralSubCmds) &&
                                peripheralSubCmds.TryGetValue(block.Bytes[0].Value, out var specificSubCmds) &&
                                specificSubCmds.TryGetValue(currentByte.Value, out var actualSubCmdName)) 
                            {
                                subCmdName = actualSubCmdName;
                            }
                            detail = subCmdName;
                            byteColor = DecoderOutputColor.Orange; // Different color for sub-command data
                        }
                        else if (isLastDataInPM)
                        {
                            detail = "LastData";
                            byteColor = DecoderOutputColor.Green;
                        }
                        else if (isChk)
                        {
                             detail = "CHK";
                             byteColor = chkOk ? DecoderOutputColor.Green : DecoderOutputColor.Red;
                             detail += chkOk ? " OK" : $" ERR (exp {calculatedChk:X2})";
                        }
                        else if (isGenericData) // General data byte
                        {
                            if (!block.IsMasterToPeripheral && currentByte.ModeBit && i == 0) // First byte of P->M, likely response
                            {
                                detail = GetPeripheralResponseName(currentByte.Value, peripheralTypeForBlock); // peripheralTypeForBlock might be from previous M->P
                                byteColor = DecoderOutputColor.Orange; // Color for responses
                            }
                            else
                            {
                                detail = "Data";
                                byteColor = blockColor;
                            }
                        }

                        // Append Mode bit info to detail (unless it's CHK or already part of command string for Addr)
                        if (!isChk && !isAddrByte) // For AddrByte, mode is implicit (must be 1)
                        {
                           detail += $" M={(currentByte.ModeBit ? 1 : 0)}";
                        }
                        else if (isAddrByte) // For AddrByte, the mode is 1. If GetMdbCommandDetails didn't add it, it's fine.
                        {
                            // Confirming mode bit is 1 for address byte, already handled by isAddrByte condition
                        }

                        outputList.Add(new DecoderOutputValueNumeric(
                            currentByte.StartIndex,
                            currentByte.EndIndex,
                            byteColor,
                            currentByte.Value,
                            detail,
                            8 // Specify 8 data bits for formatting
                            ));
                    }

                    // Add a separate event for checksum status if checksum was expected
                    /* // --- REMOVED to prevent overlapping text ---
                    if (checksumExpected)
                    {
                        outputList.Add(new DecoderOutputEvent(
                            potentialChkByte.StartIndex, // Span the checksum byte
                            potentialChkByte.EndIndex,
                            chkOk ? DecoderOutputColor.Black : DecoderOutputColor.Red,
                            chkOk ? "CHK OK" : $"CHK ERR (exp {calculatedChk:X2})"));
                    }
                    */

                    // --- Keep the 'No CHK?' marker for unexpected block endings ---
                    if (!checksumExpected && block.Bytes.Count > 1) // Check if checksum was *not* expected and block had >1 byte
                    {
                         // Indicate if checksum wasn't expected or block structure seemed wrong
                         outputList.Add(new DecoderOutputEvent(
                             block.EndIndex - (block.Bytes.LastOrDefault()?.SampleCount ?? 0), // Span the last byte
                             block.EndIndex,
                             DecoderOutputColor.Orange,
                             "No CHK?"));
                    }

                    // Remove the old overall block representation
                    // string detail = $"{blockLabel.ToString().Trim()}\n{blockTooltip.ToString()}";
                    // outputList.Add(new DecoderOutputValueNumeric(blockStartIndex, blockEndIndex, blockColor, 0, detail, 0));

                } else if (block.Bytes.Count == 1 && !firstByte.ModeBit) {
                     // Single byte with Mode=0 - likely invalid/noise
                     outputList.Add(new DecoderOutputEvent(blockStartIndex, blockEndIndex, DecoderOutputColor.Orange, $"??? (0x{firstByte.Value:X2} M=0)"));
                }


            }
        }

        /// <summary>
        /// Determines the MDB peripheral type based on the address byte.
        /// </summary>
        /// <param name="address">The address byte.</param>
        /// <returns>A string representing the peripheral type.</returns>
        private static string GetPeripheralType(byte address)
        {
            // The peripheral type is determined by the 5 most significant bits (mask 0xF8).
            byte addressBase = (byte)(address & 0xF8);

            switch (addressBase)
            {
                case 0x00: return "VMC"; // Vending Machine Controller (Master)
                case 0x08: return "Changer";
                case 0x10: return "Cashless #1";
                case 0x18: return "Comm Gateway";
                case 0x20: return "Display";
                case 0x28: return "Energy Mgmt";
                case 0x30: return "Bill Validator";
                case 0x38: return "Reserved (Future)";
                case 0x40: return "USD #1"; // Universal Satellite Device
                case 0x48: return "USD #2";
                case 0x50: return "USD #3";
                case 0x58: return "Dispenser #1"; // Coin Hopper/Tube
                case 0x60: return "Cashless #2";
                case 0x68: return "Age Verify"; // Note: Was Dispenser #2 in v4.0, changed to 70H in v4.1 for Dispenser #2
                case 0x70: return "Dispenser #2"; // Coin Hopper/Tube
                case byte n when (n >= 0x78 && n <= 0xD8): return "Reserved (Future)";
                // Specific addresses for Experimental/VMS based on user's list (E0H, E8H, F0H, F8H)
                case 0xE0: 
                case 0xE8: 
                case 0xF0: 
                case 0xF8: return "Experimental/VMS";
                default: return "Unknown Device";
            }
        }

        /// <summary>
        /// The parity (Not used in MDB, kept for reference if needed elsewhere).
        /// </summary>
        private enum Parity
        {
            None,
            Odd,
            Even,
            Mark,
            Space
        }
        
        /// <summary>
        /// The bit (Not directly used in MDB logic, can be removed or kept for debugging).
        /// </summary>
        private class Bit
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="Bit"/> class.
            /// </summary>
            /// <param name="index"> The index. </param>
            /// <param name="length"> The length. </param>
            /// <param name="val"> The val. </param>
            public Bit(int index, int length, bool val)
            {
                this.Index = index;
                this.Value = val;
                this.Length = length;
            }

            /// <summary>
            /// Gets the index.
            /// </summary>
            public int Index { get; private set; }

            /// <summary>
            /// Gets the value.
            /// </summary>
            public bool Value { get; private set; }

            /// <summary>
            /// Gets the length.
            /// </summary>
            public int Length { get; private set; }

            /// <summary>
            /// The to string.
            /// </summary>
            /// <returns>
            /// The <see cref="string"/>.
            /// </returns>
            public override string ToString()
            {
                return string.Format("{0},{1},{2}", this.Index, this.Length, this.Value);
            }
        }
    }
}