import math
import pretty_midi

midi_data = pretty_midi.PrettyMIDI("risen.mid")

freqs = []

prev_end = 0
for note in midi_data.instruments[0].notes:
    if note.end == prev_end:
        continue
    delay = note.start - prev_end
    if delay > 0:
        # print(delay)
        freqs.append(
            (0, int(round(delay * 1000, 0))),
        )
        print(f"delay: {delay}")
    print(note, pretty_midi.note_number_to_hz(note.pitch))
    freq = pretty_midi.note_number_to_hz(note.pitch)
    duration = note.end - note.start
    print(f"duration: {duration}")
    freqs.append(
        (int(round(freq, 0)), int(round(duration * 1000, 0))),
    )
    prev_end = note.end

for f in freqs:
    print(f"freq: {f[0]}, dur: {f[1]}")

print("static music_score_t hymn[] = {")
for i, (freq, duration) in enumerate(freqs):
    if i % 4:
        print("{ " + str(freq) + ", " + str(duration) + " }, ", end="")
    else:
        print("{ " + str(freq) + ", " + str(duration) + " },")
print("};")
